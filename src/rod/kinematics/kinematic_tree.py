import copy
import dataclasses
import functools
from typing import Dict, List, Sequence, Tuple, Union

import numpy as np

import rod
from rod import logging
from rod.tree import DirectedTree, DirectedTreeNode, TreeEdge, TreeFrame


@dataclasses.dataclass(frozen=True)
class KinematicTree(DirectedTree):
    model: "rod.Model"

    joints: List[TreeEdge] = dataclasses.field(default_factory=list)
    frames: List[TreeFrame] = dataclasses.field(default_factory=list)

    def __post_init__(self):
        # Initialize base class
        super().__post_init__()

        # Ge the index of the last link
        last_node_idx = list(iter(self))[-1].index

        # Assign frames indices. The frames indexing continues the link indexing.
        for frame_idx, node in enumerate(self.frames):
            node.index = last_node_idx + 1 + frame_idx

        # Assign joint indices. The joint index matches the index of its child link.
        for joint in self.joints:
            joint.index = self.links_dict[joint.child.name()].index

        # Order the lists containing joints and frames
        self.joints.sort(key=lambda j: j.index)
        self.frames.sort(key=lambda f: f.index)

    def link_names(self) -> List[str]:
        return [node.name() for node in self]

    def frame_names(self) -> List[str]:
        return [frame.name() for frame in self.frames]

    def joint_names(self) -> List[str]:
        return [joint.name() for joint in self.joints]

    @staticmethod
    def build(model: "rod.Model", is_top_level: bool = True) -> "KinematicTree":
        logging.debug(msg=f"Building kinematic tree of model '{model.name}'")

        if model.model is not None:
            msg = "Model composition is not yet supported. Ignoring all sub-models."
            logging.warning(msg=msg)

        # Copy the model and make reference frames explicit, i.e. add the pose element
        # to all models, links, joints, frames.
        # In this build method, we don't require any specific FrameConvention since
        # converting a tree to a new convention would need to build the tree first.
        model = copy.deepcopy(model)
        model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

        # Generally speaking, a rod.Model describes a DAG (directed acyclic graph).
        # Since we do not yet support closed loops / parallel kinematic structures,
        # we can restrict the family of supported rod.Models to those describing
        # a tree, i.e. a DAG whose nodes have a single parent.
        # The links of the model are the nodes of the tree, and joints its edges.
        # The root of the tree is defined by the canonical link of the model.
        # The model could also define additional elements called frames that are
        # pseudo-nodes attached to either a node or another pseudo-node (another frame).

        # In our tree, links are the nodes and joints the edges.
        # Create a dict mapping link names to tree nodes, for easy retrieval.
        nodes_links_dict: Dict[str, DirectedTreeNode] = {
            # Add one node for each link of the model
            **{link.name: DirectedTreeNode(_source=link) for link in model.links()},
            # Add special world node, that will become a frame later
            TreeFrame.WORLD: DirectedTreeNode(
                _source=rod.Link(
                    name=TreeFrame.WORLD,
                    pose=rod.Pose(relative_to=TreeFrame.WORLD),
                )
            ),
        }

        # Get the canonical link of the model.
        # The canonical link defines the implicit coordinate frame of the model,
        # and by default the implicit __model__ frame is attached to it.
        # Note: selecting the wrong canonical link would produce an invalid tree having
        #       unconnected links and joints, situation that would raise an error.
        # Note: after building the tree from the rod.Model, it will be possible to change
        #       the canonical link (also known as base link), operation that performs a
        #       tree re-balancing that would produce a new model having its __model__
        #       frame not attached to its canonical link.
        root_node_name = model.get_canonical_link()
        logging.debug(msg=f"Selecting '{root_node_name}' as canonical link")
        assert root_node_name in nodes_links_dict, root_node_name

        # Furthermore, existing frames are extra elements that could be optionally
        # attached to the kinematic tree (but by default they're not part of it).
        # Create a dict mapping frame names to frame nodes, for easy retrieval.
        nodes_frames_dict: Dict[str, TreeFrame] = {
            # Add a frame node for each frame in the model
            **{frame.name: TreeFrame(_source=frame) for frame in model.frames()},
            # Add implicit frames used in the SDF specification (__model__).
            # The following frames are attached to the first link found in the model
            # description and never moved, so that all elements expressing their pose
            # w.r.t. these frames always remain valid.
            TreeFrame.MODEL: TreeFrame(
                _source=rod.Frame(
                    name=TreeFrame.MODEL,
                    attached_to=root_node_name,
                    pose=model.pose,
                ),
            ),
        }

        # Check that links and frames have unique names
        assert len(
            set(list(nodes_links_dict.keys()) + list(nodes_frames_dict.keys()))
        ) == (len(nodes_links_dict) + len(nodes_frames_dict))

        # Use joints to connect nodes by defining their parent and children
        for joint in model.joints():
            if joint.child == TreeFrame.WORLD:
                msg = f"A joint cannot have '{TreeFrame.WORLD}' as child"
                raise RuntimeError(msg)

            # Get the parent and child nodes of the joint
            child_node = nodes_links_dict[joint.child]
            parent_node = nodes_links_dict[joint.parent]

            # Check that the dict is correct
            assert child_node.name() == joint.child, (child_node.name(), joint.child)
            assert parent_node.name() == joint.parent, (
                parent_node.name(),
                joint.parent,
            )

            # Assign to each child node their parent
            child_node.parent = parent_node

            # Assign to each node their children, and make sure they are unique
            if child_node.name() not in {n.name() for n in parent_node.children}:
                parent_node.children.append(child_node)

        # Compute the tree traversal with BFS algorithm.
        # If the model is fixed-base, the world node is not part of the tree and the
        # joint connecting to world will be removed.
        all_node_names_in_tree = [
            n.name()
            for n in list(
                KinematicTree.breadth_first_search(
                    root=nodes_links_dict[root_node_name]
                )
            )
        ]

        # Get all the joints part of the kinematic tree ...
        joints_in_tree_names = [
            j.name
            for j in model.joints()
            if {j.parent, j.child}.issubset(all_node_names_in_tree)
        ]
        joints_in_tree = [j for j in model.joints() if j.name in joints_in_tree_names]

        # ... and those that are not
        joints_not_in_tree = [
            j for j in model.joints() if j.name not in joints_in_tree_names
        ]

        # A valid rod.Model does not have any dangling link and any unconnected joints.
        # Here we check that the rod.Model contains a valid tree representation.
        found_num_extra_joints = len(joints_not_in_tree)
        expected_num_extra_joints = 1 if model.is_fixed_base() else 0

        if found_num_extra_joints != expected_num_extra_joints:
            if model.is_fixed_base() and found_num_extra_joints == 0:
                raise RuntimeError("Failed to find joint connecting the model to world")

            unexpected_joint_names = [j.name for j in joints_not_in_tree]
            raise RuntimeError(f"Found unexpected joints: {unexpected_joint_names}")

        # Handle connection to world of fixed-base models
        if model.is_fixed_base():
            assert len(joints_not_in_tree) == 1
            world_to_base_joint = joints_not_in_tree[0]

            # Create a temporary edge so that we can reuse the logic implemented for
            # the link lumping process
            world_to_base_edge = TreeEdge(
                parent=nodes_links_dict[world_to_base_joint.parent],
                child=nodes_links_dict[world_to_base_joint.child],
                _source=world_to_base_joint,
            )

            # Produce new nodes and frames by removing the edge connecting base to world.
            # One of the additional frame will be the world frame.
            new_base_node, additional_frames = KinematicTree.remove_edge(
                edge=world_to_base_edge, keep_parent=False
            )
            assert any([f.name() == TreeFrame.WORLD for f in additional_frames])

            # Replace the former base node with the new base node
            nodes_links_dict[new_base_node.name()] = new_base_node

            # Add all the additional frames created by the edge removal process
            nodes_frames_dict = {
                **nodes_frames_dict,
                **{f.name(): f for f in additional_frames},
            }

            # Remove the world node from the nodes dictionary since it was
            # converted to frame and already added to the frames dictionary
            world_node = nodes_links_dict.pop(TreeFrame.WORLD)
            assert world_node is not None

        else:
            # Remove the world node from the nodes dictionary since it's unconnected...
            world_node = nodes_links_dict.pop(TreeFrame.WORLD)

            # ... and add it as an explicit frame attached to the root node
            nodes_frames_dict[world_node.name()] = TreeFrame.from_node(
                node=world_node, attached_to=nodes_links_dict[root_node_name]
            )

        # Create an edge for all joints
        edges_dict = {
            joint.name: TreeEdge(
                parent=nodes_links_dict[joint.parent],
                child=nodes_links_dict[joint.child],
                _source=joint,
            )
            for joint in joints_in_tree
        }

        # Build the tree, it assigns indices upon construction
        tree = KinematicTree(
            root=nodes_links_dict[root_node_name],
            joints=list(edges_dict.values()),
            frames=list(nodes_frames_dict.values()),
            model=model,
        )

        return tree

    @staticmethod
    def remove_edge(
        edge: TreeEdge, keep_parent: bool = True
    ) -> Tuple[DirectedTreeNode, Sequence[TreeFrame]]:
        # Removed node: the node to remove.
        # Replaced node: the node removed and replaced with the new node.
        # New node: the new node that combines the removed and replaced nodes.

        if keep_parent:
            # Lump child into parent.
            # This is the default behavior.
            removed_node = edge.child
            replaced_node = edge.parent
            new_node = dataclasses.replace(replaced_node)

        else:
            # Lump parent into child.
            # Can be useful when lumping the special world node into the base.
            removed_node = edge.parent
            replaced_node = edge.child
            new_node = dataclasses.replace(replaced_node, parent=removed_node.parent)

        # Convert the removed edge to frame
        removed_edge_as_frame = TreeFrame.from_edge(edge=edge, attached_to=new_node)

        # Convert the removed node as frame
        removed_node_as_frame = TreeFrame.from_node(
            node=removed_node, attached_to=removed_edge_as_frame
        )

        # Create a list with all new frames resulting from the edge removal process
        new_frames = [removed_node_as_frame, removed_edge_as_frame]

        # Check if a link has non-trivial inertial parameters
        def has_zero_inertial(link: rod.Link) -> bool:
            if not isinstance(link, rod.Link):
                return True

            if link.inertial is None:
                return True

            return np.allclose(link.inertial.mass, 0.0) and np.allclose(
                link.inertial.inertia, np.zeros(shape=(3, 3))
            )

        # The new node has the same inertial parameters of the removed node if the
        # removed node has zero inertial parameters.
        # In this case, the new node is equivalent to the removed one and its name can
        # be the same. We return the new node and the two new frames (the removed node
        # -either parent or child- and the removed edge).
        if has_zero_inertial(link=removed_node._source):
            return new_node, new_frames

        # ========================
        # Lump inertial properties
        # ========================

        raise NotImplementedError("Inertial parameters lumping")

    @functools.cached_property
    def links_dict(self) -> Dict[str, DirectedTreeNode]:
        return self.nodes_dict

    @functools.cached_property
    def frames_dict(self) -> Dict[str, TreeFrame]:
        return {frame.name(): frame for frame in self.frames}

    @functools.cached_property
    def joints_dict(self) -> Dict[str, TreeEdge]:
        return {joint.name(): joint for joint in self.joints}

    @functools.cached_property
    def joints_connection_dict(self) -> Dict[Tuple[str, str], TreeEdge]:
        return {(j.parent.name(), j.child.name()): j for j in self.joints}
