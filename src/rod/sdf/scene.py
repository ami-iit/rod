import dataclasses

import mashumaro

from .element import Element


@dataclasses.dataclass
class Scene(Element):
    ambient: list[float] = dataclasses.field(
        default_factory=lambda: [0.4, 0.4, 0.4, 1],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=4),
        ),
    )

    background: list[float] = dataclasses.field(
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

    grid: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    origin_visual: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )
