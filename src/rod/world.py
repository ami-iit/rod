import dataclasses
from typing import List, Optional, Union

import mashumaro.mixins.dict

from .element import Element
from .model import Model
from .physics import Physics
from .scene import Scene


@dataclasses.dataclass
class World(Element):

    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    gravity: List[float] = dataclasses.field(
        default_factory=lambda: [0, 0, -9.8],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    magnetic_field: List[float] = dataclasses.field(
        default_factory=lambda: [6e-6, 2.3e-5, -4.2e-5],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    physics: Physics = dataclasses.field(default_factory=Physics)

    scene: Scene = dataclasses.field(default_factory=Scene)

    model: Optional[Union[Model, List[Model]]] = dataclasses.field(default=None)
