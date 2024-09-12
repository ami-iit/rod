from __future__ import annotations

import abc
import dataclasses
from typing import ClassVar

import rod
from rod import logging


class TreeElement(abc.ABC):
    index: int | None = dataclasses.field(default=None, init=False)

    @abc.abstractmethod
    def name(self) -> str:
        pass

    @abc.abstractmethod
    def pose(self) -> rod.Pose:
        pass

    def __eq__(self, other) -> bool:
        if not isinstance(other, type(self)):
            return False

        return self.name() == other.name()

    def __hash__(self):
        return hash(str(type(self)) + self.name())


@dataclasses.dataclass(eq=False)
class DirectedTreeNode(TreeElement):
    parent: DirectedTreeNode | None = None
    children: list[DirectedTreeNode] = dataclasses.field(default_factory=list)

    _source: rod.Link | None = dataclasses.field(default=None, repr=False)

    def name(self) -> str:
        return self._source.name

    def pose(self) -> rod.Pose:
        if self._source is not None and self._source.pose is not None:
            return self._source.pose

        return rod.Pose(relative_to="world")

    @property
    def tree_label(self) -> str:
        return (
            f"#{self.index}_<{self.name()}>"
            if self.index is not None
            else f"<{self.name()}>"
        )

    def __str__(self) -> str:
        content_string = (
            f"name={self.name()}, "
            f"index={self.index}, "
            f"parent={self.parent.name() if self.parent else str(None)}, "
            f"children=[{[c.name() for c in self.children]}]"
        )

        return f"{type(self).__name__}({content_string})"


@dataclasses.dataclass(eq=False)
class TreeEdge(TreeElement):
    child: DirectedTreeNode
    parent: DirectedTreeNode

    _source: rod.Joint | None = dataclasses.field(default=None, repr=False)

    def pose(self) -> rod.Pose:
        return self._source.pose

    def name(self) -> str:
        return self._source.name

    def __str__(self) -> str:
        content_string = f"name={self.name()}, parent={self.parent}, child={self.child}"

        return f"{type(self).__name__}({content_string})"


@dataclasses.dataclass(eq=False)
class TreeFrame(TreeElement):
    WORLD: ClassVar[str] = "world"
    MODEL: ClassVar[str] = "__model__"

    _source: rod.Frame | None = dataclasses.field(default=None, repr=False)

    def name(self) -> str:
        return self._source.name

    def attached_to(self) -> str:
        return self._source.attached_to

    def pose(self) -> rod.Pose:
        return self._source.pose

    def __str__(self) -> str:
        content_string = (
            f"name={self.name()}, index={self.index}, attached_to={self.attached_to()}"
        )

        return f"{type(self).__name__}({content_string})"

    @staticmethod
    def from_node(
        node: DirectedTreeNode,
        attached_to: DirectedTreeNode | TreeFrame | TreeEdge | None = None,
    ) -> TreeFrame:
        attached_to = attached_to if attached_to is not None else node.parent

        logging.debug(
            msg="Node '{}' became a frame attached to '{}'".format(
                node.name(), attached_to.name() if attached_to is not None else "None"
            )
        )

        return TreeFrame(
            _source=rod.Frame(
                name=node.name(), pose=node._source.pose, attached_to=attached_to.name()
            )
        )

    @staticmethod
    def from_edge(
        edge: TreeEdge,
        attached_to: DirectedTreeNode | TreeFrame | TreeEdge | None = None,
    ) -> TreeFrame:
        attached_to = attached_to if attached_to is not None else edge.parent

        logging.debug(
            msg="Edge '{}' became a frame attached to '{}'".format(
                edge.name(), attached_to.name() if attached_to is not None else "None"
            )
        )

        return TreeFrame(
            _source=rod.Frame(
                name=edge.name(), pose=edge._source.pose, attached_to=attached_to.name()
            )
        )
