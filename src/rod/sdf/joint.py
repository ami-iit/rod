import dataclasses

import mashumaro

from .common import Pose, Xyz
from .element import Element


@dataclasses.dataclass
class Limit(Element):

    lower: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    upper: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    effort: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    velocity: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    stiffness: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    dissipation: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )


@dataclasses.dataclass
class Dynamics(Element):
    spring_reference: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    spring_stiffness: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    damping: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    friction: float | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )


@dataclasses.dataclass
class Axis(Element):
    xyz: Xyz
    limit: Limit
    dynamics: Dynamics | None = dataclasses.field(default=None)


@dataclasses.dataclass
class Joint(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))
    type: str = dataclasses.field(metadata=mashumaro.field_options(alias="@type"))

    parent: str
    child: str

    pose: Pose | None = dataclasses.field(default=None)
    axis: Axis | None = dataclasses.field(default=None)
