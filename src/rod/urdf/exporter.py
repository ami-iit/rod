from __future__ import annotations

import abc
import copy
import dataclasses
import logging
from typing import (
    Any,
    ClassVar,
    TypeAlias,
    TypeVar,
)

import numpy as np
import xmltodict

import rod

# Type aliases for better readability
FramesList: TypeAlias = list[dict[str, Any]]
JointsList: TypeAlias = list[dict[str, Any]]
PreserveJointsOption: TypeAlias = bool | list[str]
T = TypeVar("T")  # Generic type for optional handling


@dataclasses.dataclass
class UrdfExporter(abc.ABC):
    """Resources to convert an in-memory ROD model to URDF with elegant Pythonic patterns."""

    indent: str = "  "  # String to use for each indentation level
    pretty: bool = False  # Whether to include indentation and newlines
    gazebo_preserve_fixed_joints: PreserveJointsOption = False  # Joints to preserve

    # Class constants
    SUPPORTED_JOINT_TYPES: ClassVar[set[str]] = {
        "revolute",
        "continuous",
        "prismatic",
        "fixed",
    }

    DEFAULT_MATERIAL: ClassVar[dict[str, Any]] = {
        "@name": "default_material",
        "color": {
            "@rgba": " ".join(map(str, [1, 1, 1, 1])),
        },
    }

    def to_urdf_string(self, sdf: rod.Sdf | rod.Model) -> str:
        """Convert an in-memory SDF model to a URDF string.

        Args:
            sdf: The SDF model parsed by ROD to convert.

        Returns:
            The URDF string representing the converted SDF model.
        """
        # Work with a copy to avoid modifying the original
        sdf = copy.deepcopy(sdf)

        # Get the model (handle both Sdf and Model types)
        model = self._extract_model(sdf)
        logging.debug(f"Converting model '{model.name}' to URDF")

        # Prepare the model
        self._prepare_model_for_conversion(model)

        # Process frames and get extra elements
        extra_links, extra_joints = self._process_frames(model)

        # Handle fixed joints preservation
        preserved_joints = self._get_preserved_fixed_joints(model)

        # Build and return the URDF
        return self._build_urdf_string(
            model, extra_links, extra_joints, preserved_joints
        )

    @classmethod
    def sdf_to_urdf_string(
        cls,
        sdf: rod.Sdf | rod.Model,
        pretty: bool = False,
        indent: str = "  ",
        gazebo_preserve_fixed_joints: PreserveJointsOption = False,
    ) -> str:
        """Legacy method maintained for backward compatibility."""

        logging.warning(
            "This method is deprecated, please use 'UrdfExporter.to_urdf_string' instead."
        )

        exporter = cls(
            pretty=pretty,
            indent=indent,
            gazebo_preserve_fixed_joints=gazebo_preserve_fixed_joints,
        )

        return exporter.to_urdf_string(sdf)

    def _extract_model(self, sdf: rod.Sdf | rod.Model) -> rod.Model:
        """Extract the model from an SDF object or return the model directly."""

        if isinstance(sdf, rod.Model):
            return sdf

        if len(sdf.models()) > 1:
            raise RuntimeError("URDF only supports one robot element")

        return sdf.models()[0]

    def _prepare_model_for_conversion(self, model: rod.Model) -> None:
        """Prepare the model for conversion to URDF format."""

        # Remove all poses that could be assumed being implicit
        model.resolve_frames(is_top_level=True, explicit_frames=False)

        # Handle sub-models (not supported in URDF)
        if model.models():
            logging.warning(f"Ignoring unsupported sub-models of model '{model.name}'")
            model.model = None

        # Check model pose validity
        self._validate_model_pose(model)

        # Process canonical link
        self._process_canonical_link(model)

        # Convert all poses to use the URDF frames convention
        model.switch_frame_convention(
            frame_convention=rod.FrameConvention.Urdf,
            explicit_frames=True,
            attach_frames_to_links=True,
        )

        # Clean up link poses (in URDF, links are attached to parent joint frames)
        for link in model.links():
            if link.pose is not None and not np.allclose(link.pose.pose, np.zeros(6)):
                logging.warning(f"Ignoring non-trivial pose of link '{link.name}'")
                link.pose = None

    def _validate_model_pose(self, model: rod.Model) -> None:
        """Validate the model pose for URDF compatibility."""

        if model.pose is not None and model.pose.relative_to not in {"", None}:
            raise RuntimeError("Invalid model pose")

        if (
            model.is_fixed_base()
            and model.pose is not None
            and not np.allclose(model.pose.pose, np.zeros(6))
        ):
            logging.warning("Ignoring non-trivial pose of fixed-base model")
            model.pose = None

    def _process_canonical_link(self, model: rod.Model) -> None:
        """Process the canonical link of the model."""

        canonical_link_name = model.get_canonical_link()
        logging.debug(f"Detected '{canonical_link_name}' as root link")

        link_dict = {l.name: l for l in model.links()}
        canonical_link = link_dict[canonical_link_name]

        # Check if canonical link has a custom pose
        if (
            not model.is_fixed_base()
            and canonical_link.pose is not None
            and not np.allclose(canonical_link.pose.pose, np.zeros(6))
        ):
            logging.warning(
                f"Ignoring non-trivial pose of canonical link '{canonical_link.name}'"
            )
            canonical_link.pose = None

    def _process_frames(self, model: rod.Model) -> tuple[FramesList, JointsList]:
        """Convert SDF frames to URDF equivalent chains."""

        extra_links = []
        extra_joints = []

        for frame in model.frames():
            dummy_link = self._create_dummy_link(frame)
            new_joint = self._create_dummy_joint(frame, dummy_link["@name"])

            logging.debug(
                f"Processing frame '{frame.name}': created new dummy chain "
                f"{frame.attached_to}->({new_joint['@name']})->{dummy_link['@name']}"
            )

            extra_links.append(dummy_link)
            extra_joints.append(new_joint)

        return extra_links, extra_joints

    def _create_dummy_link(self, frame: rod.Frame) -> dict[str, Any]:
        """Create a dummy link for a frame."""

        return {
            "@name": frame.name,
            "inertial": {
                "origin": {"@xyz": "0 0 0", "@rpy": "0 0 0"},
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

    def _create_dummy_joint(
        self, frame: rod.Frame, dummy_link_name: str
    ) -> dict[str, Any]:
        """Create a dummy joint for a frame."""

        # The pose of the frame in FrameConvention.Urdf refers to the parent link
        assert frame.pose.relative_to == frame.attached_to

        return {
            "@name": f"{frame.attached_to}_to_{dummy_link_name}",
            "@type": "fixed",
            "parent": {"@link": frame.attached_to},
            "child": {"@link": dummy_link_name},
            "origin": {
                "@xyz": " ".join(np.array(frame.pose.xyz, dtype=str)),
                "@rpy": " ".join(np.array(frame.pose.rpy, dtype=str)),
            },
        }

    def _get_preserved_fixed_joints(self, model: rod.Model) -> list[str]:
        """Get the list of fixed joints to preserve."""

        preserve_option = copy.copy(self.gazebo_preserve_fixed_joints)

        # Convert boolean option to list of joint names
        if preserve_option is True:
            preserve_option = [j.name for j in model.joints() if j.type == "fixed"]
        elif preserve_option is False:
            preserve_option = []

        assert isinstance(preserve_option, list)

        # Validate that all specified joints exist
        model_joint_names = {j.name for j in model.joints()}
        for joint_name in preserve_option:
            logging.debug(f"Preserving fixed joint '{joint_name}'")
            if joint_name not in model_joint_names:
                raise RuntimeError(f"Joint '{joint_name}' not found in the model")

        return preserve_option

    def _build_urdf_string(
        self,
        model: rod.Model,
        extra_links: FramesList,
        extra_joints: JointsList,
        preserved_joints: list[str],
    ) -> str:
        """Build the URDF string from the model and additional elements."""

        # Define world link for fixed-base models
        world_link = rod.Link(name="world")
        world_link_dict = [world_link.to_dict()] if model.is_fixed_base() else []

        # Create the URDF dictionary
        urdf_dict = {
            "robot": {
                "@name": model.name,
                "link": world_link_dict
                + self._create_link_elements(model)
                + extra_links,
                "joint": self._create_joint_elements(model) + extra_joints,
                "gazebo": [
                    {
                        "@reference": fixed_joint,
                        "preserveFixedJoint": "true",
                        "disableFixedJointLumping": "true",
                    }
                    for fixed_joint in preserved_joints
                ],
            }
        }

        # Convert to XML string
        return xmltodict.unparse(
            input_dict=urdf_dict,
            pretty=self.pretty,
            indent=self.indent,
            short_empty_elements=True,
        )

    def _create_link_elements(self, model: rod.Model) -> list[dict[str, Any]]:
        """Create the link elements for the URDF."""

        return [
            {
                "@name": link.name,
                "inertial": self._create_inertial_element(link),
                "visual": self._create_visual_elements(link),
                "collision": self._create_collision_elements(link),
            }
            for link in model.links()
        ]

    def _create_inertial_element(self, link: rod.Link) -> dict[str, Any]:
        """Create an inertial element for a link."""

        return {
            "origin": {
                "@xyz": " ".join(map(str, link.inertial.pose.xyz)),
                "@rpy": " ".join(map(str, link.inertial.pose.rpy)),
            },
            "mass": {"@value": link.inertial.mass},
            "inertia": {
                "@ixx": link.inertial.inertia.ixx,
                "@ixy": link.inertial.inertia.ixy,
                "@ixz": link.inertial.inertia.ixz,
                "@iyy": link.inertial.inertia.iyy,
                "@iyz": link.inertial.inertia.iyz,
                "@izz": link.inertial.inertia.izz,
            },
        }

    def _create_visual_elements(self, link: rod.Link) -> list[dict[str, Any]]:
        """Create visual elements for a link."""

        return [
            {
                "@name": visual.name,
                "origin": {
                    "@xyz": " ".join(map(str, visual.pose.xyz)),
                    "@rpy": " ".join(map(str, visual.pose.rpy)),
                },
                "geometry": self._rod_geometry_to_xmltodict(visual.geometry),
                **(
                    {"material": self._rod_material_to_xmltodict(visual.material)}
                    if visual.material is not None
                    else {}
                ),
            }
            for visual in link.visuals()
        ]

    def _create_collision_elements(self, link: rod.Link) -> list[dict[str, Any]]:
        """Create collision elements for a link."""

        return [
            {
                "@name": collision.name,
                "origin": {
                    "@xyz": " ".join(map(str, collision.pose.xyz)),
                    "@rpy": " ".join(map(str, collision.pose.rpy)),
                },
                "geometry": self._rod_geometry_to_xmltodict(collision.geometry),
            }
            for collision in link.collisions()
        ]

    def _create_joint_elements(self, model: rod.Model) -> list[dict[str, Any]]:
        """Create the joint elements for the URDF."""

        return [
            self._joint_to_dict(joint)
            for joint in model.joints()
            if joint.type in self.SUPPORTED_JOINT_TYPES
        ]

    def _joint_to_dict(self, joint: rod.Joint) -> dict[str, Any]:
        """Convert a joint to a dictionary representation."""

        joint_dict = {
            "@name": joint.name,
            "@type": joint.type,
            "origin": {
                "@xyz": " ".join(map(str, joint.pose.xyz)),
                "@rpy": " ".join(map(str, joint.pose.rpy)),
            },
            "parent": {"@link": joint.parent},
            "child": {"@link": joint.child},
        }

        # Add axis if needed and not a fixed joint
        if (
            joint.axis is not None
            and joint.axis.xyz is not None
            and joint.type != "fixed"
        ):
            joint_dict["axis"] = {"@xyz": " ".join(map(str, joint.axis.xyz.xyz))}

        # Add dynamics if available and not a fixed joint
        if (
            joint.axis is not None
            and joint.axis.dynamics is not None
            and {joint.axis.dynamics.damping, joint.axis.dynamics.friction} != {None}
            and joint.type != "fixed"
        ):

            dynamics = {}
            if joint.axis.dynamics.damping is not None:
                dynamics["@damping"] = joint.axis.dynamics.damping
            if joint.axis.dynamics.friction is not None:
                dynamics["@friction"] = joint.axis.dynamics.friction

            if dynamics:
                joint_dict["dynamics"] = dynamics

        # Add limits if needed and not a fixed joint
        limit_dict = {}

        if (
            joint.axis is not None
            and joint.axis.limit is not None
            and joint.type in {"revolute", "prismatic"}
        ):

            # Effort and velocity get defaults if not specified
            limit_dict["@effort"] = self._get_or_default(
                joint.axis.limit.effort, np.finfo(np.float32).max
            )
            limit_dict["@velocity"] = self._get_or_default(
                joint.axis.limit.velocity, np.finfo(np.float32).max
            )

            # Lower and upper only for revolute and prismatic joints
            if joint.type in {"revolute", "prismatic"}:
                if joint.axis.limit.lower is not None:
                    limit_dict["@lower"] = joint.axis.limit.lower
                if joint.axis.limit.upper is not None:
                    limit_dict["@upper"] = joint.axis.limit.upper

        joint_dict["limit"] = limit_dict

        return joint_dict

    @staticmethod
    def _get_or_default(value: T | None, default: T) -> T:
        """Return the value if not None, otherwise the default."""

        return value if value is not None else default

    @staticmethod
    def _rod_geometry_to_xmltodict(geometry: rod.Geometry) -> dict[str, Any]:
        """Convert ROD geometry to XML dictionary format."""

        result = {}

        if geometry.box is not None:
            result["box"] = {"@size": " ".join(np.array(geometry.box.size, dtype=str))}
        elif geometry.cylinder is not None:
            result["cylinder"] = {
                "@radius": geometry.cylinder.radius,
                "@length": geometry.cylinder.length,
            }
        elif geometry.sphere is not None:
            result["sphere"] = {"@radius": geometry.sphere.radius}
        elif geometry.mesh is not None:
            result["mesh"] = {
                "@filename": geometry.mesh.uri,
                "@scale": " ".join(map(str, geometry.mesh.scale)),
            }

        return result

    @classmethod
    def _rod_material_to_xmltodict(cls, material: rod.Material) -> dict[str, Any]:
        """Convert ROD material to XML dictionary format."""

        if material.script is not None:
            logging.info(
                "Material scripts are not supported, returning default material"
            )
            return cls.DEFAULT_MATERIAL

        if material.diffuse is None:
            logging.info(
                "Material diffuse color is not defined, returning default material"
            )
            return cls.DEFAULT_MATERIAL

        return {
            "@name": f"color_{hash(' '.join(map(str, material.diffuse)))}",
            "color": {
                "@rgba": " ".join(map(str, material.diffuse)),
            },
        }
