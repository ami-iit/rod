import dataclasses
import pathlib
from typing import Union

import trimesh
from numpy.typing import NDArray

import rod
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
    mesh_path: Union[str, pathlib.Path]
    scale: NDArray

    def __post_init__(self) -> None:
        """
        Post-initialization method for the class.
        Loads the mesh from the specified file path and performs necessary checks.

        Raises:
            AssertionError: If the scale is not a 3D vector.
            TypeError: If the mesh_path is not a str or pathlib.Path.
        """

        if isinstance(self.mesh_path, str):
            extension = pathlib.Path(self.mesh_path).suffix
        elif isinstance(self.mesh_path, pathlib.Path):
            extension = self.mesh_path.suffix
        else:
            raise TypeError(
                f"Expected str or pathlib.Path for mesh_path, got {type(self.mesh_path)}"
            )

        self.mesh: trimesh.base.Trimesh = trimesh.load(
            str(self.mesh_path),
            force="mesh",
            file_type=extension,
        )

        assert self.scale.shape == (
            3,
        ), f"Scale must be a 3D vector, got {self.scale.shape}"

    def _inertia(self) -> rod.Inertia:
        inertia = self.mesh.moment_inertia
        return rod.Inertia(
            ixx=inertia[0, 0],
            ixy=inertia[0, 1],
            ixz=inertia[0, 2],
            iyy=inertia[1, 1],
            iyz=inertia[1, 2],
            izz=inertia[2, 2],
        )

    def _geometry(self) -> rod.Geometry:
        return rod.Geometry(mesh=rod.Mesh(uri=str(self.mesh_path), scale=self.scale))
