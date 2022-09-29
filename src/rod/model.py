import dataclasses
from typing import List, Optional, Union

import mashumaro

from .common import Pose
from .element import Element
from .joint import Joint
from .link import Link


@dataclasses.dataclass
class Model(Element):

    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    canonical_link: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@canonical_link")
    )

    placement_frame: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@placement_frame")
    )

    static: Optional[bool] = dataclasses.field(
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

    allow_auto_disable: Optional[bool] = dataclasses.field(
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

    pose: Optional[Pose] = dataclasses.field(default=None)

    model: Optional[Union["Model", List["Model"]]] = dataclasses.field(default=None)

    link: Optional[Union[Link, List[Link]]] = dataclasses.field(default=None)

    joint: Optional[Union[Joint, List[Joint]]] = dataclasses.field(default=None)
