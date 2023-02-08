import dataclasses
from typing import List, Optional

import mashumaro

from .element import Element


@dataclasses.dataclass
class Script(Element):
    name: str
    uri: str = dataclasses.field(default="__default__")


@dataclasses.dataclass
class Material(Element):
    script: Optional[Script] = dataclasses.field(default=None)

    lightning: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    ambient: Optional[List[float]] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    diffuse: Optional[List[float]] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    specular: Optional[List[float]] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    emissive: Optional[List[float]] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )
