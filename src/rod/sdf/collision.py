import dataclasses

import mashumaro

from .common import Pose
from .element import Element
from .geometry import Geometry


@dataclasses.dataclass
class Collision(Element):
    geometry: Geometry
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    pose: Pose | None = dataclasses.field(default=None)
