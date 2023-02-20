import dataclasses
from typing import List, Optional, Union

import mashumaro
import numpy as np
import numpy.typing as npt

from .collision import Collision
from .common import Pose
from .element import Element
from .visual import Visual


@dataclasses.dataclass
class Inertia(Element):
    ixx: float = dataclasses.field(
        default=1.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    iyy: float = dataclasses.field(
        default=1.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    izz: float = dataclasses.field(
        default=1.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    ixy: float = dataclasses.field(
        default=0.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    ixz: float = dataclasses.field(
        default=0.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    iyz: float = dataclasses.field(
        default=0.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    def matrix(self) -> npt.NDArray:
        return np.array(
            [
                [self.ixx, self.ixy, self.ixz],
                [self.ixy, self.iyy, self.iyz],
                [self.ixz, self.iyz, self.izz],
            ]
        )


@dataclasses.dataclass
class Inertial(Element):
    mass: float = dataclasses.field(
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    inertia: Inertia

    name: Optional[str] = dataclasses.field(default=None)
    pose: Optional[Pose] = dataclasses.field(default=None)


@dataclasses.dataclass
class Link(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    pose: Optional[Pose] = dataclasses.field(default=None)

    inertial: Optional[Inertial] = dataclasses.field(default=None)

    visual: Optional[Union[Visual, List[Visual]]] = dataclasses.field(default=None)

    collision: Optional[Union[Collision, List[Collision]]] = dataclasses.field(
        default=None
    )

    gravity: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    enable_wind: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    self_collide: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    kinematic: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    must_be_base_link: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    def visuals(self) -> List[Visual]:
        if self.visual is None:
            return []

        if isinstance(self.visual, Visual):
            return [self.visual]

        assert isinstance(self.visual, list), type(self.visual)
        return self.visual

    def collisions(self) -> List[Collision]:
        if self.collision is None:
            return []

        if isinstance(self.collision, Collision):
            return [self.collision]

        assert isinstance(self.collision, list), type(self.collision)
        return self.collision

    def add_visual(self, visual: Visual) -> None:
        if self.visual is None:
            self.visual = visual
            return

        visuals = self.visual

        if not isinstance(visuals, list):
            assert isinstance(visuals, Visual), type(visuals)
            visuals = [visuals]

        visuals.append(visual)
        self.visual = visuals

    def add_collision(self, collision: Collision) -> None:
        if self.collision is None:
            self.collision = collision
            return

        collisions = self.collision

        if not isinstance(collisions, list):
            assert isinstance(collisions, Collision), type(collisions)
            collisions = [collisions]

        collisions.append(collision)
        self.collision = collisions
