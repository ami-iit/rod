from __future__ import annotations

import dataclasses

import mashumaro
import numpy as np
import numpy.typing as npt

from rod import logging

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

    @staticmethod
    def from_inertia_tensor(
        inertia_tensor: npt.NDArray, validate: bool = True
    ) -> Inertia:

        inertia_tensor = inertia_tensor.squeeze()

        if inertia_tensor.shape != (3, 3):
            raise ValueError(f"Expected shape (3, 3), got {inertia_tensor.shape}")

        # Extract the diagonal terms.
        I1, I2, I3 = np.diag(inertia_tensor)

        # Check if the inertia tensor meets the triangular inequality.
        valid = True
        valid = valid and I1 + I2 >= I3
        valid = valid and I1 + I3 >= I2
        valid = valid and I2 + I3 >= I1

        if not valid:
            msg = "Inertia tensor does not meet the triangular inequality"

            if not validate:
                logging.warning(msg)
            else:
                raise ValueError(msg)

        return Inertia(
            ixx=float(inertia_tensor[0, 0]),
            ixy=float(inertia_tensor[0, 1]),
            ixz=float(inertia_tensor[0, 2]),
            iyy=float(inertia_tensor[1, 1]),
            iyz=float(inertia_tensor[1, 2]),
            izz=float(inertia_tensor[2, 2]),
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

    name: str | None = dataclasses.field(default=None)
    pose: Pose | None = dataclasses.field(default=None)


@dataclasses.dataclass
class Link(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    pose: Pose | None = dataclasses.field(default=None)

    inertial: Inertial | None = dataclasses.field(default=None)

    visual: Visual | list[Visual] | None = dataclasses.field(default=None)

    collision: Collision | list[Collision] | None = dataclasses.field(default=None)

    gravity: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    enable_wind: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    self_collide: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    kinematic: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    must_be_base_link: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    def visuals(self) -> list[Visual]:
        if self.visual is None:
            return []

        if isinstance(self.visual, Visual):
            return [self.visual]

        assert isinstance(self.visual, list), type(self.visual)
        return self.visual

    def collisions(self) -> list[Collision]:
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
