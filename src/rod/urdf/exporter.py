import abc
import copy
import dataclasses
from typing import Any, ClassVar, Set

import numpy as np
import xmltodict

import rod
from rod import logging


@dataclasses.dataclass
class UrdfExporter(abc.ABC):
    """Resources to convert an in-memory ROD model to URDF."""

    # The string to use for each indentation level.
    indent: str = "  "

    # Whether to include indentation and newlines in the output.
    pretty: bool = False

    # Whether to inject additional `<gazebo>` elements in the resulting URDF
    # to preserve fixed joints in case of re-loading into sdformat.
    # If a list of strings is passed, only the listed fixed joints will be preserved.
    gazebo_preserve_fixed_joints: bool | list[str] = False

    SupportedSdfJointTypes: ClassVar[Set[str]] = {
        "revolute",
        "continuous",
        "prismatic",
        "fixed",
    }

    DefaultMaterial: ClassVar[dict[str, Any]] = {
        "@name": "default_material",
        "color": {
            "@rgba": " ".join(np.array([1, 1, 1, 1], dtype=str)),
        },
    }

    @staticmethod
    def sdf_to_urdf_string(
        sdf: rod.Sdf | rod.Model,
        pretty: bool = False,
        indent: str = "  ",
        gazebo_preserve_fixed_joints: bool | list[str] = False,
    ) -> str:

        msg = "This method is deprecated, please use '{}' instead."
        logging.warning(msg.format("UrdfExporter.to_urdf_string"))

        return UrdfExporter(
            pretty=pretty,
            indent=indent,
            gazebo_preserve_fixed_joints=gazebo_preserve_fixed_joints,
        ).to_urdf_string(sdf=sdf)

    def to_urdf_string(self, sdf: rod.Sdf | rod.Model) -> str:
        """
        Convert an in-memory SDF model to a URDF string.

        Args:
            sdf: The SDF model parsed by ROD to convert.

        Returns:
            The URDF string representing the converted SDF model.
        """

        # Operate on a copy of the sdf object
        sdf = copy.deepcopy(sdf)

        if isinstance(sdf, rod.Sdf) and len(sdf.models()) > 1:
            raise RuntimeError("URDF only supports one robot element")

        # Get the model
        model = sdf if isinstance(sdf, rod.Model) else sdf.models()[0]
        logging.debug(f"Converting model '{model.name}' to URDF")

        # Remove all poses that could be assumed being implicit
        model.resolve_frames(is_top_level=True, explicit_frames=False)

        # Model composition is not supported, ignoring sub-models
        if len(model.models()) > 0:
            msg = f"Ignoring unsupported sub-models of model '{model.name}'"
            logging.warning(msg=msg)

            model.model = None

        # Check that the model pose has no reference frame (implicit frame is world)
        if model.pose is not None and model.pose.relative_to not in {"", None}:
            raise RuntimeError("Invalid model pose")

        # If the model pose is not zero, warn that it will be ignored.
        # In fact, the pose wrt world of the canonical link (base) will be used instead.
        if (
            model.is_fixed_base()
            and model.pose is not None
            and not np.allclose(model.pose.pose, np.zeros(6))
        ):
            logging.warning("Ignoring non-trivial pose of fixed-base model")
            model.pose = None

        # Get the canonical link of the model
        logging.debug(f"Detected '{model.get_canonical_link()}' as root link")
        canonical_link: rod.Link = {l.name: l for l in model.links()}[
            model.get_canonical_link()
        ]

        # If the canonical link has a custom pose, notify that it will be ignored.
        # In fact, it might happen that the canonical link has a custom pose w.r.t.
        # the __model__ frame. In SDF, the __model__frame defines the default reference
        # of a model, instead in URDF this reference is represented by the root link
        # (that is, by definition, the SDF canonical link).
        if (
            not model.is_fixed_base()
            and canonical_link.pose is not None
            and not np.allclose(canonical_link.pose.pose, np.zeros(6))
        ):
            msg = "Ignoring non-trivial pose of canonical link '{name}'"
            logging.warning(msg.format(name=canonical_link.name))
            canonical_link.pose = None

        # Convert all poses to use the Urdf frames convention.
        # This process drastically simplifies extracting compatible kinematic transforms.
        # Furthermore, it post-processes frames such that they get directly attached to
        # a real link (instead of being attached to other frames).
        model.switch_frame_convention(
            frame_convention=rod.FrameConvention.Urdf,
            explicit_frames=True,
            attach_frames_to_links=True,
        )

        # ============================================
        # Convert SDF frames to URDF equivalent chains
        # ============================================

        # Initialize the containers of extra links and joints
        extra_links_from_frames: list[dict[str, Any]] = []
        extra_joints_from_frames: list[dict[str, Any]] = []

        # Since URDF does not support plain frames as SDF, we convert all frames
        # to (fixed_joint->dummy_link) sequences
        for frame in model.frames():

            # New dummy link with same name of the frame
            dummy_link = {
                "@name": frame.name,
                "inertial": {
                    "origin": {
                        "@xyz": "0 0 0",
                        "@rpy": "0 0 0",
                    },
                    "mass": {"@value": 0.0},
                    "inertia": {
                        "@ixx": 0.0,
                        "@ixy": 0.0,
                        "@ixz": 0.0,
                        "@iyy": 0.0,
                        "@iyz": 0.0,
                        "@izz": 0.0,
                    },
                },
            }

            # Note: the pose of the frame in FrameConvention.Urdf already
            # refers to the parent link, so we can directly use it.
            assert frame.pose.relative_to == frame.attached_to

            # New joint connecting the link to which the frame is attached
            # to the new dummy link.
            new_joint = {
                "@name": f"{frame.attached_to}_to_{dummy_link['@name']}",
                "@type": "fixed",
                "parent": {"@link": frame.attached_to},
                "child": {"@link": dummy_link["@name"]},
                "origin": {
                    "@xyz": " ".join(np.array(frame.pose.xyz, dtype=str)),
                    "@rpy": " ".join(np.array(frame.pose.rpy, dtype=str)),
                },
            }

            logging.debug(
                "Processing frame '{}': created new dummy chain {}->({})->{}".format(
                    frame.name,
                    frame.attached_to,
                    new_joint["@name"],
                    dummy_link["@name"],
                )
            )

            extra_links_from_frames.append(dummy_link)
            extra_joints_from_frames.append(new_joint)

        # =====================
        # Preserve fixed joints
        # =====================

        # This attribute could either be list of fixed joint names to preserve,
        # or a boolean to preserve all fixed joints.
        gazebo_preserve_fixed_joints = copy.copy(self.gazebo_preserve_fixed_joints)

        # If it is a boolean, automatically populate the list with all fixed joints.
        if gazebo_preserve_fixed_joints is True:
            gazebo_preserve_fixed_joints = [
                j.name for j in model.joints() if j.type == "fixed"
            ]

        if gazebo_preserve_fixed_joints is False:
            gazebo_preserve_fixed_joints = []

        assert isinstance(gazebo_preserve_fixed_joints, list)

        # Check that all fixed joints to preserve are actually present in the model.
        for fixed_joint_name in gazebo_preserve_fixed_joints:
            logging.debug(f"Preserving fixed joint '{fixed_joint_name}'")
            all_model_joint_names = {j.name for j in model.joints()}
            if fixed_joint_name not in all_model_joint_names:
                raise RuntimeError(f"Joint '{fixed_joint_name}' not found in the model")

        # ===================
        # Convert SDF to URDF
        # ===================

        # In URDF, links are directly attached to the frame of their parent joint
        for link in model.links():
            if link.pose is not None and not np.allclose(link.pose.pose, np.zeros(6)):
                msg = "Ignoring non-trivial pose of link '{name}'"
                logging.warning(msg.format(name=link.name))
                link.pose = None

        # Define the 'world' link used for fixed-base models
        world_link = rod.Link(name="world")

        # Create a new dict in xmldict format with only the elements supported by URDF
        urdf_dict = dict(
            robot={
                **{"@name": model.name},
                # http://wiki.ros.org/urdf/XML/link
                "link": ([world_link.to_dict()] if model.is_fixed_base() else [])
                + [
                    {
                        "@name": l.name,
                        "inertial": {
                            "origin": {
                                "@xyz": " ".join(
                                    np.array(l.inertial.pose.xyz, dtype=str)
                                ),
                                "@rpy": " ".join(
                                    np.array(l.inertial.pose.rpy, dtype=str)
                                ),
                            },
                            "mass": {"@value": l.inertial.mass},
                            "inertia": {
                                "@ixx": l.inertial.inertia.ixx,
                                "@ixy": l.inertial.inertia.ixy,
                                "@ixz": l.inertial.inertia.ixz,
                                "@iyy": l.inertial.inertia.iyy,
                                "@iyz": l.inertial.inertia.iyz,
                                "@izz": l.inertial.inertia.izz,
                            },
                        },
                        "visual": [
                            {
                                "@name": v.name,
                                "origin": {
                                    "@xyz": " ".join(np.array(v.pose.xyz, dtype=str)),
                                    "@rpy": " ".join(np.array(v.pose.rpy, dtype=str)),
                                },
                                "geometry": UrdfExporter._rod_geometry_to_xmltodict(
                                    geometry=v.geometry
                                ),
                                **(
                                    {
                                        "material": UrdfExporter._rod_material_to_xmltodict(
                                            material=v.material
                                        )
                                    }
                                    if v.material is not None
                                    else dict()
                                ),
                            }
                            for v in l.visuals()
                        ],
                        "collision": [
                            {
                                "@name": c.name,
                                "origin": {
                                    "@xyz": " ".join(np.array(c.pose.xyz, dtype=str)),
                                    "@rpy": " ".join(np.array(c.pose.rpy, dtype=str)),
                                },
                                "geometry": UrdfExporter._rod_geometry_to_xmltodict(
                                    geometry=c.geometry
                                ),
                            }
                            for c in l.collisions()
                        ],
                    }
                    for l in model.links()
                ]
                # Add the extra links resulting from the frame->dummy_link conversion
                + extra_links_from_frames,
                # http://wiki.ros.org/urdf/XML/joint
                "joint": [
                    {
                        "@name": j.name,
                        "@type": j.type,
                        "origin": {
                            "@xyz": " ".join(np.array(j.pose.xyz, dtype=str)),
                            "@rpy": " ".join(np.array(j.pose.rpy, dtype=str)),
                        },
                        "parent": {"@link": j.parent},
                        "child": {"@link": j.child},
                        **(
                            {
                                "axis": {
                                    "@xyz": " ".join(
                                        np.array(j.axis.xyz.xyz, dtype=str)
                                    )
                                }
                            }
                            if j.axis is not None
                            and j.axis.xyz is not None
                            and j.type != "fixed"
                            else dict()
                        ),
                        # calibration: does not have any SDF corresponding element
                        **(
                            {
                                "dynamics": {
                                    **(
                                        {"@damping": j.axis.dynamics.damping}
                                        if j.axis.dynamics.damping is not None
                                        else dict()
                                    ),
                                    **(
                                        {"@friction": j.axis.dynamics.friction}
                                        if j.axis.dynamics.friction is not None
                                        else dict()
                                    ),
                                }
                            }
                            if j.axis is not None
                            and j.axis.dynamics is not None
                            and {j.axis.dynamics.damping, j.axis.dynamics.friction}
                            != {None}
                            and j.type != "fixed"
                            else dict()
                        ),
                        **(
                            {
                                "limit": {
                                    **(
                                        {"@effort": j.axis.limit.effort}
                                        if j.axis.limit.effort is not None
                                        else {"@effort": np.finfo(np.float32).max}
                                    ),
                                    **(
                                        {"@velocity": j.axis.limit.velocity}
                                        if j.axis.limit.velocity is not None
                                        else {"@velocity": np.finfo(np.float32).max}
                                    ),
                                    **(
                                        {"@lower": j.axis.limit.lower}
                                        if j.axis.limit.lower is not None
                                        and j.type in {"revolute", "prismatic"}
                                        else dict()
                                    ),
                                    **(
                                        {"@upper": j.axis.limit.upper}
                                        if j.axis.limit.upper is not None
                                        and j.type in {"revolute", "prismatic"}
                                        else dict()
                                    ),
                                },
                            }
                            if j.axis is not None
                            and j.axis.limit is not None
                            and j.type != "fixed"
                            else dict()
                        ),
                        # mimic: does not have any SDF corresponding element
                        # safety_controller: does not have any SDF corresponding element
                    }
                    for j in model.joints()
                    if j.type in UrdfExporter.SupportedSdfJointTypes
                ]
                # Add the extra joints resulting from the frame->link conversion
                + extra_joints_from_frames,
                # Extra gazebo-related elements
                # https://classic.gazebosim.org/tutorials?tut=ros_urdf
                # https://github.com/gazebosim/sdformat/issues/199#issuecomment-622127508
                "gazebo": [
                    {
                        "@reference": fixed_joint,
                        "preserveFixedJoint": "true",
                        "disableFixedJointLumping": "true",
                    }
                    for fixed_joint in gazebo_preserve_fixed_joints
                ],
            }
        )

        return xmltodict.unparse(
            input_dict=urdf_dict,
            pretty=self.pretty,
            indent=self.indent,
            short_empty_elements=True,
        )

    @staticmethod
    def _rod_geometry_to_xmltodict(geometry: rod.Geometry) -> dict[str, Any]:
        return {
            **(
                {"box": {"@size": " ".join(np.array(geometry.box.size, dtype=str))}}
                if geometry.box is not None
                else dict()
            ),
            **(
                {
                    "cylinder": {
                        "@radius": geometry.cylinder.radius,
                        "@length": geometry.cylinder.length,
                    }
                }
                if geometry.cylinder is not None
                else dict()
            ),
            **(
                {"sphere": {"@radius": geometry.sphere.radius}}
                if geometry.sphere is not None
                else dict()
            ),
            **(
                {
                    "mesh": {
                        "@filename": geometry.mesh.uri,
                        "@scale": " ".join(np.array(geometry.mesh.scale, dtype=str)),
                    }
                }
                if geometry.mesh is not None
                else dict()
            ),
        }

    @staticmethod
    def _rod_material_to_xmltodict(material: rod.Material) -> dict[str, Any]:
        if material.script is not None:
            msg = "Material scripts are not supported, returning default material"
            logging.info(msg=msg)
            return UrdfExporter.DefaultMaterial

        if material.diffuse is None:
            msg = "Material diffuse color is not defined, returning default material"
            logging.info(msg=msg)
            return UrdfExporter.DefaultMaterial

        return {
            "@name": f"color_{hash(' '.join(np.array(material.diffuse, dtype=str)))}",
            "color": {
                "@rgba": " ".join(np.array(material.diffuse, dtype=str)),
            },
            # "texture": {"@filename": None},  # TODO
        }
