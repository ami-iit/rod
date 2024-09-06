import dataclasses

import mashumaro

from .common import Frame
from .element import Element
from .model import Model
from .physics import Physics
from .scene import Scene


@dataclasses.dataclass
class World(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    gravity: list[float] = dataclasses.field(
        default_factory=lambda: [0, 0, -9.8],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    magnetic_field: list[float] = dataclasses.field(
        default_factory=lambda: [6e-6, 2.3e-5, -4.2e-5],
        metadata=mashumaro.field_options(
            serialize=Element.serialize_list,
            deserialize=lambda l: Element.deserialize_list(data=l, length=3),
        ),
    )

    physics: Physics = dataclasses.field(default_factory=Physics)

    scene: Scene = dataclasses.field(default_factory=Scene)

    model: Model | list[Model] | None = dataclasses.field(default=None)

    frame: Frame | list[Frame] | None = dataclasses.field(default=None)

    def models(self) -> list[Model]:
        if self.model is None:
            return []

        if isinstance(self.model, Model):
            return [self.model]

        assert isinstance(self.model, list)
        return self.model

    def frames(self) -> list[Frame]:
        if self.frame is None:
            return []

        if isinstance(self.frame, Frame):
            return [self.frame]

        assert isinstance(self.frame, list)
        return self.frame
