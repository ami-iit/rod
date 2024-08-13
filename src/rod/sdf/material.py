import dataclasses

import mashumaro

from .element import Element


@dataclasses.dataclass
class Script(Element):
    name: str
    uri: str = dataclasses.field(default="__default__")


@dataclasses.dataclass
class Material(Element):
    script: Script | None = dataclasses.field(default=None)

    lightning: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    ambient: list[float] | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    diffuse: list[float] | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    specular: list[float] | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    emissive: list[float] | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )
