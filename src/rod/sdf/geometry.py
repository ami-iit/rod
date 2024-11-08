from __future__ import annotations

import dataclasses
import types
from typing import ClassVar

import mashumaro

from rod import logging

from .element import Element


@dataclasses.dataclass
class Box(Element):
    size: list[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )


@dataclasses.dataclass
class Capsule(Element):
    radius: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    length: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )


@dataclasses.dataclass
class Cylinder(Element):
    radius: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    length: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )


@dataclasses.dataclass
class Ellipsoid(Element):
    radii: list[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )


@dataclasses.dataclass
class Heightmap(Element):
    uri: str

    size: list[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    pos: list[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )


@dataclasses.dataclass
class Mesh(Element):
    uri: str

    scale: list[float] | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )


@dataclasses.dataclass
class Plane(Element):
    normal: list[float] = dataclasses.field(
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    size: list[float] | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=2),
        ),
    )


@dataclasses.dataclass
class Sphere(Element):
    radius: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )


@dataclasses.dataclass
class Geometry(Element):

    GeometryType: ClassVar[types.UnionType] = (
        Box | Capsule | Cylinder | Ellipsoid | Heightmap | Mesh | Plane | Sphere
    )

    box: Box | None = dataclasses.field(default=None)
    capsule: Capsule | None = dataclasses.field(default=None)
    cylinder: Cylinder | None = dataclasses.field(default=None)
    ellipsoid: Ellipsoid | None = dataclasses.field(default=None)
    heightmap: Heightmap | None = dataclasses.field(default=None)
    mesh: Mesh | None = dataclasses.field(default=None)
    plane: Plane | None = dataclasses.field(default=None)
    sphere: Sphere | None = dataclasses.field(default=None)

    def geometries(self) -> list[Geometry.GeometryType]:

        return [
            self.__getattribute__(field.name)
            for field in dataclasses.fields(self)
            if self.__getattribute__(field.name) is not None
        ]

    def geometry(self) -> Geometry.GeometryType | None:
        """Return the actual geometry stored in the object"""

        geometries = self.geometries()

        if len(geometries) > 1:
            msg = "More than one geometry found, returning the first one"
            logging.warning(msg)

        return geometries[0] if len(geometries) > 0 else None
