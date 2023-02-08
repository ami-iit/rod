import abc
import dataclasses
from typing import Optional, Union

import numpy as np
import numpy.typing as npt

import rod
from rod import logging


@dataclasses.dataclass
class PrimitiveBuilder(abc.ABC):
    name: str
    mass: float

    element: Union[
        rod.Model, rod.Link, rod.Inertial, rod.Collision, rod.Visual
    ] = dataclasses.field(
        default=None, init=False, repr=False, hash=False, compare=False
    )

    def build(
        self,
    ) -> Union[rod.Model, rod.Link, rod.Inertial, rod.Collision, rod.Visual]:
        return self.element

    # ================
    # Abstract methods
    # ================

    @abc.abstractmethod
    def _inertia(self) -> rod.Inertia:
        pass

    @abc.abstractmethod
    def _geometry(self) -> rod.Geometry:
        pass

    # ================
    # Element builders
    # ================

    def build_model(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> "PrimitiveBuilder":
        self._check_element()

        self.element = self._model(name=name, pose=pose)

        return self

    def build_link(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> "PrimitiveBuilder":
        self._check_element()

        self.element = self._link(name=name, pose=pose)

        return self

    def build_inertial(self, pose: Optional[rod.Pose] = None) -> "PrimitiveBuilder":
        self._check_element()

        self.element = self._inertial(pose=pose)

        return self

    def build_visual(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> "PrimitiveBuilder":
        self._check_element()

        self.element = self._visual(name=name, pose=pose)

        return self

    def build_collision(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> "PrimitiveBuilder":
        self._check_element()

        self.element = self._collision(name=name, pose=pose)

        return self

    # =================
    # Element modifiers
    # =================

    def add_link(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
        link: Optional[rod.Link] = None,
    ) -> "PrimitiveBuilder":
        if not isinstance(self.element, rod.Model):
            raise ValueError(type(self.element))

        link = link if link is not None else self._link(name=name, pose=pose)

        if pose is not None:
            link.pose = pose

        self.element.link = link

        return self

    def add_inertial(
        self,
        pose: Optional[rod.Pose] = None,
        inertial: Optional[rod.Inertial] = None,
    ) -> "PrimitiveBuilder":
        if not isinstance(self.element, (rod.Model, rod.Link)):
            raise ValueError(type(self.element))

        if isinstance(self.element, rod.Model):
            link = self.element.link

        elif isinstance(self.element, rod.Link):
            link = self.element

        else:
            raise ValueError(self.element)

        inertial = inertial if inertial is not None else self._inertial(pose=pose)

        if pose is not None:
            inertial.pose = pose
        else:
            inertial.pose = PrimitiveBuilder.build_pose(relative_to=link.name)

        link.inertial = inertial

        return self

    def add_visual(
        self,
        name: Optional[str] = None,
        use_inertial_pose: bool = True,
        pose: Optional[rod.Pose] = None,
        visual: Optional[rod.Visual] = None,
    ) -> "PrimitiveBuilder":
        if not isinstance(self.element, (rod.Model, rod.Link)):
            raise ValueError(type(self.element))

        if isinstance(self.element, rod.Model):
            link = self.element.link

        elif isinstance(self.element, rod.Link):
            link = self.element

        else:
            raise ValueError(self.element)

        if pose is None and use_inertial_pose:
            if link.inertial.pose is None:
                msg = f"Inertial element of link '{link.name}' has no pose defined"
                raise ValueError(msg)

            pose = link.inertial.pose

        visual = visual if visual is not None else self._visual(name=name, pose=pose)

        if visual.name in [v.name for v in link.visuals()]:
            msg = f"Visual '{visual.name}' already exists in link '{link.name}'"
            raise ValueError(msg)

        link.add_visual(visual=visual)

        return self

    def add_collision(
        self,
        name: Optional[str] = None,
        use_inertial_pose: bool = True,
        pose: Optional[rod.Pose] = None,
        collision: Optional[rod.Collision] = None,
    ) -> "PrimitiveBuilder":
        if not isinstance(self.element, (rod.Model, rod.Link)):
            raise ValueError(type(self.element))

        if isinstance(self.element, rod.Model):
            link = self.element.link

        elif isinstance(self.element, rod.Link):
            link = self.element

        else:
            raise ValueError(self.element)

        if pose is None and use_inertial_pose:
            if link.inertial.pose is None:
                msg = f"Inertial element of link '{link.name}' has no pose defined"
                raise ValueError(msg)

            pose = link.inertial.pose

        collision = (
            collision
            if collision is not None
            else self._collision(name=name, pose=pose)
        )

        if collision.name in [c.name for c in link.collisions()]:
            msg = f"Collision '{collision.name}' already exists in link '{link.name}'"
            raise ValueError(msg)

        link.add_collision(collision=collision)

        return self

    # ====================
    # ROD element builders
    # ====================

    def _model(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> rod.Model:
        name = name if name is not None else self.name
        logging.debug(f"Building model '{name}'")

        if pose is not None and pose.relative_to != "world":
            raise ValueError("Model pose must be relative to 'world")

        return rod.Model(
            name=name,
            pose=pose,
        )

    def _link(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> rod.Link:
        return rod.Link(
            name=name if name is not None else f"{self.name}_link",
            pose=pose,
        )

    def _inertial(self, pose: Optional[rod.Pose] = None) -> rod.Inertial:
        return rod.Inertial(
            pose=pose,
            mass=self.mass,
            inertia=self._inertia(),
        )

    def _visual(
        self,
        name: Optional[str] = None,
        pose: Optional[rod.Pose] = None,
    ) -> rod.Visual:
        name = name if name is not None else f"{self.name}_visual"

        return rod.Visual(
            name=name,
            pose=pose,
            geometry=self._geometry(),
        )

    def _collision(
        self,
        name: Optional[str],
        pose: Optional[rod.Pose] = None,
    ) -> rod.Collision:
        name = name if name is not None else f"{self.name}_collision"

        return rod.Collision(
            name=name,
            pose=pose,
            geometry=self._geometry(),
        )

    # ===============
    # Utility methods
    # ===============

    def _check_element(self) -> None:
        if self.element is not None:
            msg = f"Builder was already building a '{type(self.element)}' instance"
            raise ValueError(msg)

    @staticmethod
    def build_pose(
        pos: npt.NDArray = None,
        rpy: npt.NDArray = None,
        relative_to: str = None,
        degrees: bool = None,
        rotation_format: str = None,
    ) -> Optional[rod.Pose]:
        if pos is None and rpy is None:
            return rod.Pose.from_transform(transform=np.eye(4), relative_to=relative_to)

        pos = np.zeros(3) if pos is None else pos
        rpy = np.zeros(3) if rpy is None else rpy

        if pos.size != 3:
            raise ValueError(pos.size)

        if rpy.size != 3:
            raise ValueError(rpy.size)

        return rod.Pose(
            pose=list(np.hstack([pos, rpy])),
            relative_to=relative_to,
            degrees=degrees,
            rotation_format=rotation_format,
        )
