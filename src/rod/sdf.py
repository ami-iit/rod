import dataclasses
from typing import List, Optional, Union

import mashumaro

from .element import Element
from .model import Model
from .world import World


@dataclasses.dataclass
class Sdf(Element):

    version: str = dataclasses.field(metadata=mashumaro.field_options(alias="@version"))

    world: Optional[Union[World, List[World]]] = dataclasses.field(default=None)

    model: Optional[Union[Model, List[Model]]] = dataclasses.field(default=None)
