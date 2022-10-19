import dataclasses
from typing import Any, Dict, List, Optional

import mashumaro

from .element import Element


@dataclasses.dataclass
class Xyz(Element):

    xyz: List[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    expressed_in: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@expressed_in")
    )

    @classmethod
    def __pre_deserialize__(cls, d: Dict[Any, Any]) -> Dict[Any, Any]:

        if isinstance(d, str):
            d = {"#text": d, "@expressed_in": ""}

        return d


@dataclasses.dataclass
class Pose(Element):

    pose: List[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="#text",
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=6),
        ),
    )

    relative_to: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@relative_to")
    )

    degrees: Optional[bool] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@degrees")
    )

    rotation_format: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@rotation_format")
    )

    @classmethod
    def __pre_deserialize__(cls, d: Dict[Any, Any]) -> Dict[Any, Any]:

        if isinstance(d, str):
            d = {"#text": d, "@relative_to": ""}

        return d

    @property
    def xyz(self) -> List[float]:
        return self.pose[0:3]

    @property
    def rpy(self) -> List[float]:
        return self.pose[3:6]


@dataclasses.dataclass
class Frame(Element):

    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    attached_to: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@attached_to")
    )

    pose: Optional[Pose] = dataclasses.field(default=None)
