import dataclasses
from typing import List, Optional

import mashumaro

from .element import Element


@dataclasses.dataclass
class Scene(Element):
    ambient: List[float] = dataclasses.field(
        default_factory=lambda: [0.4, 0.4, 0.4, 1],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    background: List[float] = dataclasses.field(
        default_factory=lambda: [0.7, 0.7, 0.7, 1],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    shadows: bool = dataclasses.field(
        default=True,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    grid: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    origin_visual: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )
