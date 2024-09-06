import dataclasses

import mashumaro

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
    box: Box | None = dataclasses.field(default=None)
    capsule: Capsule | None = dataclasses.field(default=None)
    cylinder: Capsule | None = dataclasses.field(default=None)
    ellipsoid: Capsule | None = dataclasses.field(default=None)
    heightmap: Heightmap | None = dataclasses.field(default=None)
    mesh: Mesh | None = dataclasses.field(default=None)
    plane: Mesh | None = dataclasses.field(default=None)
    sphere: Sphere | None = dataclasses.field(default=None)
