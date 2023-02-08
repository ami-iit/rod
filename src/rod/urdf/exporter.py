import abc
import copy
from typing import Any, Dict, List, Union

import numpy as np
import xmltodict

import rod
from rod import logging
from rod.kinematics.tree_transforms import TreeTransforms


class UrdfExporter(abc.ABC):
    SUPPORTED_SDF_JOINT_TYPES = {"revolute", "continuous", "prismatic", "fixed"}

    @staticmethod
    def sdf_to_urdf_string(
        sdf: rod.Sdf,
        pretty: bool = False,
        indent: str = "  ",
        gazebo_preserve_fixed_joints: Union[bool, List[str]] = False,
    ) -> str:
        # Operate on a copy of the sdf object
        sdf = copy.deepcopy(sdf)

        if len(sdf.models()) > 1:
            raise RuntimeError("URDF only supports one robot element")

        # Get the model
        model = sdf.models()[0]

        # Remove all poses that could be assumed being implicit
        model.resolve_frames(is_top_level=True, explicit_frames=False)

        # Model composition is not supported, ignoring sub-models
        if len(model.models()) > 0:
            msg = f"Ignoring unsupported sub-models of model '{model.name}'"
            logging.warning(msg=msg)

            model.models = None

        # Check that the model pose has no reference frame (implicit frame is world)
        if model.pose is not None and model.pose.relative_to not in {"", None}:
            raise RuntimeError("Invalid model pose")

        # If the model pose is not zero, warn that it will be ignored.
        # In fact, the pose wrt world of the canonical link will be used instead.
        if (
            model.is_fixed_base()
            and model.pose is not None
            and not np.allclose(model.pose.pose, np.zeros(6))
        ):
            logging.warning("Ignoring non-trivial pose of fixed-base model")
            model.pose = None

        # Get the canonical link of the model
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
        # This process drastically simplifies extracting compatible kinematic trasforms.
        model.switch_frame_convention(
            frame_convention=rod.FrameConvention.Urdf, explicit_frames=True
        )

        # ============================================
        # Convert SDF frames to URDF equivalent chains
        # ============================================

        # Tree transforms helper used to process SDF frame poses, if any.
        # No need to switch frame convention to Urdf since it was already done above.
        tree_transforms = (
            TreeTransforms.build(
                model=model,
                is_top_level=True,
                prevent_switching_frame_convention=True,
            )
            if len(model.frames()) is not None
            else None
        )

        # Initialize the containers of extra links and joints
        extra_links_from_frames: List[Dict[str, Any]] = []
        extra_joints_from_frames: List[Dict[str, Any]] = []

        # Since URDF does not support plain frames as SDF, we convert all frames
        # to (fixed_joint->dummy_link) sequences
        for frame in model.frames():
            # Find the name of the first parent link of a frame (since a frame could be
            # attached to another frame). Falls back to __model__ in case of failure.
            parent_link_name = UrdfExporter._find_parent_link(frame=frame, model=model)

            # Populate a new Pose with the transform between the link to which the fixed
            # joint will be attached and the frame that will be converted to dummy link
            new_link_pose = rod.Pose.from_transform(
                transform=tree_transforms.relative_transform(
                    relative_to=parent_link_name, name=frame.name
                ),
                relative_to=parent_link_name,
            )

            # Smallest value not ignored by sdformat
            epsilon = 1e-6 + 1e-9

            # New dummy link with same name of the frame
            new_link = {
                "@name": frame.name,
                # If the link has no inertial properties, parsing again the exported
                # URDF model with SDF would ignore it.
                # We need to add a tiny fake mass greater than 1e-6.
                # https://github.com/gazebosim/sdformat/issues/199#issuecomment-622127508
                "inertial": {
                    "origin": {
                        "@xyz": "0 0 0",
                        "@rpy": "0 0 0",
                    },
                    "mass": {"@value": str(epsilon)},
                    "inertia": {
                        "@ixx": str(epsilon),
                        "@ixy": 0,
                        "@ixz": 0,
                        "@iyy": str(epsilon),
                        "@iyz": 0,
                        "@izz": str(epsilon),
                    },
                },
                "visual": {
                    "@name": f"{frame.name}_visual",
                    "origin": {
                        "@xyz": "0 0 0",
                        "@rpy": "0 0 0",
                    },
                    "geometry": UrdfExporter._rod_geometry_to_xmltodict(
                        geometry=rod.Geometry(sphere=rod.Sphere(radius=0.0))
                    ),
                },
            }

            # New joint connecting the detected parent link to the new link
            new_joint = {
                "@name": f"{parent_link_name}_to_{new_link['@name']}",
                "@type": "fixed",
                "parent": {"@link": parent_link_name},
                "child": {"@link": new_link["@name"]},
                "origin": {
                    "@xyz": " ".join(np.array(new_link_pose.xyz, dtype=str)),
                    "@rpy": " ".join(np.array(new_link_pose.rpy, dtype=str)),
                },
            }

            extra_links_from_frames.append(new_link)
            extra_joints_from_frames.append(new_joint)

        # =====================
        # Preserve fixed joints
        # =====================

        if gazebo_preserve_fixed_joints is True:
            gazebo_preserve_fixed_joints = [
                j.name for j in model.joints() if j.type == "fixed"
            ]

        if gazebo_preserve_fixed_joints is False:
            gazebo_preserve_fixed_joints = []

        assert isinstance(gazebo_preserve_fixed_joints, list)

        # ===================
        # Convert SDF to URDF
        # ===================

        for link in model.links():
            if link.pose is not None and not np.allclose(link.pose.pose, np.zeros(6)):
                msg = "Ignoring non-trivial pose of link '{name}'"
                logging.warning(msg.format(name=link.name))
                link.pose = None

        # Define the world link used for fixed-base models
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
                                        "material": {
                                            # TODO: add colors logic
                                            "@name": "white",
                                            "color": {
                                                "@rgba": " ".join(
                                                    np.array([1, 1, 1, 0], dtype=str)
                                                )
                                            },
                                            # TODO: add textures support
                                            # "texture": {"@filename": None},
                                        }
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
                # Add the extra links resulting from the frame->link conversion
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
                    if j.type in UrdfExporter.SUPPORTED_SDF_JOINT_TYPES
                ]
                # Add the extra joints resulting from the frame->link conversion
                + extra_joints_from_frames,
                # Extra gazebo-related elements
                # https://classic.gazebosim.org/tutorials?tut=ros_urdf
                # https://github.com/gazebosim/sdformat/issues/199#issuecomment-622127508
                "gazebo": [
                    {
                        "@reference": extra_joint["@name"],
                        "preserveFixedJoint": "true",
                        "disableFixedJointLumping": "true",
                    }
                    for extra_joint in extra_joints_from_frames
                ]
                + [
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
            pretty=pretty,
            indent=indent,
            short_empty_elements=True,
        )

    @staticmethod
    def _find_parent_link(frame: rod.Frame, model: rod.Model) -> str:
        links_dict = {l.name: l for l in model.links()}
        frames_dict = {f.name: f for f in model.frames()}
        joints_dict = {j.name: j for j in model.joints()}
        sub_models_dict = {m.name: m for m in model.models()}

        parent = frame

        # SDF frames can be attached to links, joints, the model, or other frames.
        # - link: consider it as parent
        # - joint: consider its child as parent (child because is rigidly attached)
        # - model: consider the canonical link as parent
        # - frame: recursive call to find its parent
        while True:
            if isinstance(parent, rod.Frame) and not parent.name == frame.name:
                return UrdfExporter._find_parent_link(frame=parent, model=model)

            if isinstance(parent, rod.Link):
                return parent.name

            if isinstance(parent, rod.Joint):
                return parent.child

            parent_name = frame.attached_to

            if parent_name in links_dict:
                parent = links_dict[parent_name]

            elif parent_name in joints_dict:
                parent = joints_dict[parent_name]

            elif parent_name in frames_dict:
                parent = frames_dict[parent_name]

            elif parent_name == model.name:
                return "__model__"

            elif parent_name in sub_models_dict:
                raise RuntimeError("Model composition not yet supported")

            else:
                raise RuntimeError(f"Failed to find element with name '{parent_name}'")

    @staticmethod
    def _rod_geometry_to_xmltodict(geometry: rod.Geometry) -> Dict[str, Any]:
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
