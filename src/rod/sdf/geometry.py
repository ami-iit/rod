import dataclasses
from typing import List, Optional

import mashumaro

from .element import Element


@dataclasses.dataclass
class Box(Element):
    size: List[float] = dataclasses.field(
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
    radii: List[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )


@dataclasses.dataclass
class Heightmap(Element):
    uri: str

    size: List[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    pos: List[float] = dataclasses.field(
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

    scale: Optional[List[float]] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )


@dataclasses.dataclass
class Plane(Element):
    normal: List[float] = dataclasses.field(
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    size: Optional[List[float]] = dataclasses.field(
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
    box: Optional[Box] = dataclasses.field(default=None)
    capsule: Optional[Capsule] = dataclasses.field(default=None)
    cylinder: Optional[Capsule] = dataclasses.field(default=None)
    ellipsoid: Optional[Capsule] = dataclasses.field(default=None)
    heightmap: Optional[Heightmap] = dataclasses.field(default=None)
    mesh: Optional[Mesh] = dataclasses.field(default=None)
    plane: Optional[Mesh] = dataclasses.field(default=None)
    sphere: Optional[Sphere] = dataclasses.field(default=None)
