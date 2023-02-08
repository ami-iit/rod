import dataclasses
from typing import Optional

import mashumaro

from .common import Pose, Xyz
from .element import Element


@dataclasses.dataclass
class Limit(Element):
    lower: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float)
    )

    upper: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float)
    )

    effort: Optional[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    velocity: Optional[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    stiffness: Optional[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    dissipation: Optional[float] = dataclasses.field(
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

    damping: Optional[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    friction: Optional[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )


@dataclasses.dataclass
class Axis(Element):
    xyz: Xyz
    limit: Limit
    dynamics: Optional[Dynamics] = dataclasses.field(default=None)


@dataclasses.dataclass
class Joint(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))
    type: str = dataclasses.field(metadata=mashumaro.field_options(alias="@type"))

    parent: str
    child: str

    pose: Optional[Pose] = dataclasses.field(default=None)
    axis: Optional[Axis] = dataclasses.field(default=None)
