import dataclasses
from typing import Optional

import mashumaro

from .common import Pose
from .element import Element
from .geometry import Geometry
from .material import Material


@dataclasses.dataclass
class Visual(Element):
    geometry: Geometry

    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    pose: Optional[Pose] = dataclasses.field(default=None)

    material: Optional[Material] = dataclasses.field(default=None)
