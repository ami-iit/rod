from __future__ import annotations

import dataclasses
from typing import Any

import mashumaro
import numpy.typing as npt

from .element import Element


@dataclasses.dataclass
class Xyz(Element):
    xyz: list[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    expressed_in: str | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@expressed_in")
    )

    @classmethod
    def __pre_deserialize__(cls, d: dict[Any, Any]) -> dict[Any, Any]:
        if isinstance(d, str):
            d = {"#text": d, "@expressed_in": ""}

        return d


@dataclasses.dataclass
class Pose(Element):
    pose: list[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=6),
        ),
    )

    relative_to: str | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@relative_to")
    )

    degrees: bool | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@degrees")
    )

    rotation_format: str | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@rotation_format")
    )

    @classmethod
    def __pre_deserialize__(cls, d: dict[Any, Any]) -> dict[Any, Any]:
        if isinstance(d, str):
            d = {"#text": d, "@relative_to": ""}

        return d

    @property
    def xyz(self) -> list[float]:
        return self.pose[0:3]

    @property
    def rpy(self) -> list[float]:
        return self.pose[3:6]

    def transform(self) -> npt.NDArray:
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        # Transform Euler angles to DCM matrix.
        # The rpy sequence included in URDF and SDF implements the x-y-z Tait-Bryan
        # angles using the extrinsic convention (w.r.t. a fixed frame).
        DCM = R.from_euler(
            seq="xyz",
            angles=self.rpy,
            degrees=self.degrees if self.degrees is True else False,
        ).as_matrix()

        return np.block(
            [
                [DCM, np.vstack(self.xyz)],
                [0, 0, 0, 1.0],
            ]
        )

    @staticmethod
    def from_transform(transform: npt.NDArray, relative_to: str | None = None) -> Pose:
        if transform.shape != (4, 4):
            raise ValueError(transform.shape)

        from scipy.spatial.transform import Rotation as R

        xyz = list(transform[0:3, 3].squeeze())
        rpy = list(R.from_matrix(transform[0:3, 0:3]).as_euler(seq="xyz"))

        return Pose(pose=xyz + rpy, relative_to=relative_to)


@dataclasses.dataclass
class Frame(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    attached_to: str | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@attached_to")
    )

    pose: Pose | None = dataclasses.field(default=None)
