from __future__ import annotations

import dataclasses

import mashumaro

import rod
from rod import logging

from .common import Frame, Pose
from .element import Element
from .joint import Joint
from .link import Link


@dataclasses.dataclass
class Model(Element):
    name: str = dataclasses.field(metadata=mashumaro.field_options(alias="@name"))

    canonical_link: str | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@canonical_link")
    )

    placement_frame: str | None = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@placement_frame")
    )

    static: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    self_collide: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    allow_auto_disable: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    enable_wind: bool | None = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            serialize=Element.serialize_bool, deserialize=Element.deserialize_bool
        ),
    )

    pose: Pose | None = dataclasses.field(default=None)

    model: Model | list[Model] | None = dataclasses.field(default=None)

    frame: Frame | list[Frame] | None = dataclasses.field(default=None)

    link: Link | list[Link] | None = dataclasses.field(default=None)

    joint: Joint | list[Joint] | None = dataclasses.field(default=None)

    def is_fixed_base(self) -> bool:
        joints_having_world_parent = [j for j in self.joints() if j.parent == "world"]
        assert len(joints_having_world_parent) in {0, 1}

        return len(joints_having_world_parent) > 0

    def get_canonical_link(self) -> str:
        if len(self.models()) != 0:
            msg = "Model composition is not yet supported."
            msg += " The returned canonical link could be wrong."
            logging.warning(msg=msg)

        if self.canonical_link is not None:
            assert self.canonical_link in {l.name for l in self.links()}
            return self.canonical_link

        return self.links()[0].name

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

    def links(self) -> list[Link]:
        if self.link is None:
            return []

        if isinstance(self.link, Link):
            return [self.link]

        assert isinstance(self.link, list), type(self.link)
        return self.link

    def joints(self) -> list[Joint]:
        if self.joint is None:
            return []

        if isinstance(self.joint, Joint):
            return [self.joint]

        assert isinstance(self.joint, list), type(self.joint)
        return self.joint

    def add_frame(self, frame: Frame) -> None:
        if self.frame is None:
            self.frame = frame
            return

        frames = self.frame

        if not isinstance(frames, list):
            assert isinstance(frames, Frame)
            frames = [frames]

        frames.append(frame)
        self.frame = frames

    def resolve_uris(self) -> None:
        from rod.utils import resolve_uris

        for link in self.links():
            for visual in link.visuals():
                resolve_uris.resolve_geometry_uris(geometry=visual.geometry)

            for collision in link.collisions():
                resolve_uris.resolve_geometry_uris(geometry=collision.geometry)

    def resolve_frames(
        self, is_top_level: bool = True, explicit_frames: bool = True
    ) -> None:
        from rod.utils import resolve_frames

        resolve_frames.resolve_model_frames(
            model=self, is_top_level=is_top_level, explicit_frames=explicit_frames
        )

    def switch_frame_convention(
        self,
        frame_convention: rod.FrameConvention,
        is_top_level: bool = True,
        explicit_frames: bool = True,
        attach_frames_to_links: bool = True,
    ) -> None:
        from rod.utils.frame_convention import switch_frame_convention

        switch_frame_convention(
            model=self,
            frame_convention=frame_convention,
            is_top_level=is_top_level,
            attach_frames_to_links=attach_frames_to_links,
        )

        self.resolve_frames(is_top_level=is_top_level, explicit_frames=explicit_frames)
