import dataclasses
import pathlib

import numpy as np
import numpy.typing as npt
import resolve_robotics_uri_py
import trimesh

import rod
from rod import logging
from rod.builder.primitive_builder import PrimitiveBuilder


@dataclasses.dataclass
class SphereBuilder(PrimitiveBuilder):
    radius: float

    def _inertia(self) -> rod.Inertia:
        return rod.Inertia(
            ixx=2 / 5 * self.mass * (self.radius) ** 2,
            iyy=2 / 5 * self.mass * (self.radius) ** 2,
            izz=2 / 5 * self.mass * (self.radius) ** 2,
        )

    def _geometry(self) -> rod.Geometry:
        return rod.Geometry(sphere=rod.Sphere(radius=self.radius))


@dataclasses.dataclass
class BoxBuilder(PrimitiveBuilder):
    x: float
    y: float
    z: float

    def _inertia(self) -> rod.Inertia:
        return rod.Inertia(
            ixx=self.mass / 12 * (self.y**2 + self.z**2),
            iyy=self.mass / 12 * (self.x**2 + self.z**2),
            izz=self.mass / 12 * (self.x**2 + self.y**2),
        )

    def _geometry(self) -> rod.Geometry:
        return rod.Geometry(box=rod.Box(size=[self.x, self.y, self.z]))


@dataclasses.dataclass
class CylinderBuilder(PrimitiveBuilder):
    radius: float
    length: float

    def _inertia(self) -> rod.Inertia:
        ixx_iyy = self.mass * (3 * self.radius**2 + self.length**2) / 12

        return rod.Inertia(
            ixx=ixx_iyy,
            iyy=ixx_iyy,
            izz=0.5 * self.mass * self.radius**2,
        )

    def _geometry(self) -> rod.Geometry:
        return rod.Geometry(
            cylinder=rod.Cylinder(radius=self.radius, length=self.length)
        )


@dataclasses.dataclass
class MeshBuilder(PrimitiveBuilder):

    mesh_uri: str | pathlib.Path

    scale: npt.NDArray = dataclasses.field(default_factory=lambda: np.ones(3))

    mass: float | None = dataclasses.field(default=None, kw_only=True)
    inertia_tensor: npt.NDArray | None = dataclasses.field(default=None, kw_only=True)

    def __post_init__(self) -> None:
        """
        Post-initialization method for the class.
        Loads the mesh from the specified file path and performs necessary checks.

        Raises:
            AssertionError: If the scale is not a 3D vector.
            TypeError: If the mesh_path is not a str or pathlib.Path.
        """
        # Adjust the shape of the scale.
        self.scale = self.scale.squeeze()

        if self.scale.shape != (3,):
            raise RuntimeError(f"Scale must be a 3D vector, got '{self.scale.shape}'")

        # Resolve the mesh URI.
        mesh_path = resolve_robotics_uri_py.resolve_robotics_uri(uri=str(self.mesh_uri))

        # Build the trimesh object from the mesh path.
        self.mesh: trimesh.base.Trimesh = trimesh.load_mesh(file_obj=mesh_path)

        # Populate the mass from the mesh if it was not provided externally.
        if self.mass is None:

            if self.mesh.is_watertight:
                self.mass = self.mesh.mass

            else:
                msg = "Mesh '{}' is not watertight. Using a dummy mass value."
                logging.warning(msg.format(self.mesh_uri))
                self.mass = 1.0

        # Populate the inertia tensor from the mesh if it was not provided externally.
        if self.inertia_tensor is None:

            if self.mesh.is_watertight:
                self.inertia_tensor = self.mesh.moment_inertia

            else:
                msg = "Mesh '{}' is not watertight. Using a dummy inertia tensor."
                logging.warning(msg.format(self.mesh_uri))
                self.inertia_tensor = np.eye(3)

    def _inertia(self) -> rod.Inertia:

        return rod.Inertia.from_inertia_tensor(inertia_tensor=self.inertia_tensor)

    def _geometry(self) -> rod.Geometry:

        return rod.Geometry(
            mesh=rod.Mesh(uri=str(self.mesh_uri), scale=self.scale.tolist())
        )
