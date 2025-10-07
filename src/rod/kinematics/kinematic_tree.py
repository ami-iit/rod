from __future__ import annotations

import copy
import dataclasses
import functools

import numpy as np

import rod
from rod import logging
from rod.tree import DirectedTree, DirectedTreeNode, TreeEdge, TreeFrame


@dataclasses.dataclass(frozen=True)
class KinematicTree(DirectedTree):
    model: rod.Model
    joints: list[TreeEdge] = dataclasses.field(default_factory=list)
    frames: list[TreeFrame] = dataclasses.field(default_factory=list)

    def __post_init__(self):
        super().__post_init__()
        self._assign_indices()

    def _assign_indices(self):
        """Assign indices to frames and joints for fast access."""

        last_node_idx = len(self) - 1

        # Assign frames indices. The frames indexing continues the link indexing.
        for idx, frame in enumerate(self.frames):
            frame.index = last_node_idx + 1 + idx

        # Assign joint indices. The joint index matches the index of its child link.
        for joint in self.joints:
            joint.index = self.links_dict[joint.child.name()].index

    def link_names(self) -> list[str]:
        return [node.name() for node in self]

    def frame_names(self) -> list[str]:
        return [frame.name() for frame in self.frames]

    def joint_names(self) -> list[str]:
        return [joint.name() for joint in self.joints]

    @staticmethod
    def build(model: rod.Model, is_top_level: bool = True) -> KinematicTree:

        logging.debug(f"Building kinematic tree for model '{model.name}'")
        model = KinematicTree._prepare_model(model, is_top_level)
        nodes_links_dict, nodes_frames_dict = KinematicTree._create_nodes(model)
        edges_dict = KinematicTree._create_edges(model, nodes_links_dict)

        return KinematicTree(
            root=nodes_links_dict[model.get_canonical_link()],
            joints=list(edges_dict.values()),
            frames=list(nodes_frames_dict.values()),
            model=model,
        )

    @staticmethod
    def _prepare_model(model: rod.Model, is_top_level: bool) -> rod.Model:
        """Prepare the model by resolving frames and ensuring tree structure."""

        model = copy.deepcopy(model)
        model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

        if model.model:
            logging.warning("Model composition not supported. Ignoring sub-models.")
            model.model = None

        return model

    @staticmethod
    def _create_nodes(
        model: rod.Model,
    ) -> tuple[dict[str, DirectedTreeNode], dict[str, TreeFrame]]:
        """Create nodes for links and frames."""

        nodes_links_dict = {
            link.name: DirectedTreeNode(_source=link) for link in model.links()
        }
        nodes_links_dict[TreeFrame.WORLD] = DirectedTreeNode(
            _source=rod.Link(
                name=TreeFrame.WORLD, pose=rod.Pose(relative_to=TreeFrame.WORLD)
            )
        )
        nodes_frames_dict = {
            frame.name: TreeFrame(_source=frame) for frame in model.frames()
        }

        nodes_frames_dict[TreeFrame.MODEL] = TreeFrame(
            _source=rod.Frame(
                name=TreeFrame.MODEL,
                attached_to=model.get_canonical_link(),
                pose=model.pose,
            )
        )

        return nodes_links_dict, nodes_frames_dict

    @staticmethod
    def _create_edges(
        model: rod.Model, nodes_links_dict: dict[str, DirectedTreeNode]
    ) -> dict[str, TreeEdge]:
        """Create edges (joints) connecting nodes."""

        edges_dict = {}

        for joint in model.joints():

            if joint.child == TreeFrame.WORLD:
                raise RuntimeError(f"Joint cannot have '{TreeFrame.WORLD}' as child")

            parent_node = nodes_links_dict[joint.parent]
            child_node = nodes_links_dict[joint.child]
            child_node.parent = parent_node
            parent_node.children.append(child_node)
            edges_dict[joint.name] = TreeEdge(
                parent=parent_node, child=child_node, _source=joint
            )

        return edges_dict

    @staticmethod
    def remove_edge(
        edge: TreeEdge, keep_parent: bool = True
    ) -> tuple[DirectedTreeNode, list[TreeFrame]]:
        """Remove an edge and lump inertial properties."""

        removed_node, replaced_node = (
            (edge.child, edge.parent) if keep_parent else (edge.parent, edge.child)
        )
        new_node = dataclasses.replace(
            replaced_node, parent=removed_node.parent if not keep_parent else None
        )
        removed_edge_as_frame = TreeFrame.from_edge(edge=edge, attached_to=new_node)
        removed_node_as_frame = TreeFrame.from_node(
            node=removed_node, attached_to=removed_edge_as_frame
        )
        new_frames = [removed_node_as_frame, removed_edge_as_frame]

        if KinematicTree._has_zero_inertial(removed_node._source):
            return new_node, new_frames

        raise NotImplementedError("Inertial parameters lumping")

    @staticmethod
    def _has_zero_inertial(link: rod.Link) -> bool:
        """Check if a link has zero inertial parameters."""
        if not isinstance(link, rod.Link) or link.inertial is None:
            return True
        return np.allclose(link.inertial.mass, 0.0) and np.allclose(
            link.inertial.inertia, np.zeros((3, 3))
        )

    @functools.cached_property
    def links_dict(self) -> dict[str, DirectedTreeNode]:
        return self.nodes_dict

    @functools.cached_property
    def frames_dict(self) -> dict[str, TreeFrame]:
        return {frame.name(): frame for frame in self.frames}

    @functools.cached_property
    def joints_dict(self) -> dict[str, TreeEdge]:
        return {joint.name(): joint for joint in self.joints}

    @functools.cached_property
    def joints_connection_dict(self) -> dict[tuple[str, str], TreeEdge]:
        return {(j.parent.name(), j.child.name()): j for j in self.joints}
