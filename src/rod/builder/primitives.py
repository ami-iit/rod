import dataclasses

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
