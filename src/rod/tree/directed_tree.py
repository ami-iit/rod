import collections.abc
import dataclasses
import functools
from typing import Any, Callable, Dict, Iterable, List, Optional, Union

from .tree_elements import DirectedTreeNode


@dataclasses.dataclass(frozen=True)
class DirectedTree(collections.abc.Sequence):
    root: DirectedTreeNode

    def __post_init__(self):
        node_names = self.nodes_dict.keys()

        # We perform many tree operations using the node name as unique identifier.
        # Therefore, the node name must be unique.
        if len(node_names) != len(set(node_names)):
            raise RuntimeError("Nodes of a directed tree must have unique names")

        # Assign node indices. The node index starts with 0 assigned to the root node.
        for node_idx, node in enumerate(self):
            node.index = node_idx

    @functools.cached_property
    def nodes(self) -> List[DirectedTreeNode]:
        return list(iter(self))

    @functools.cached_property
    def nodes_dict(self) -> Dict[str, DirectedTreeNode]:
        return {node.name(): node for node in iter(self)}

    @staticmethod
    def breadth_first_search(
        root: DirectedTreeNode,
        sort_children: Optional[Callable[[Any], Any]] = lambda node: node.name(),
    ) -> Iterable[DirectedTreeNode]:
        queue = [root]

        # We assume that nodes have a unique name, and we mark a node as visited by
        # storing its name. This assumption speeds up considerably object comparison.
        visited = list()
        visited.append(root.name)

        yield root

        while len(queue) > 0:
            node = queue.pop(0)

            # Note: sorting the nodes with their name so that the order of children
            #       insertion does not matter when assigning the node index
            for child in sorted(node.children, key=sort_children):
                if child.name in visited:
                    continue

                visited.append(child.name)
                queue.append(child)

                yield child

    def pretty_print(self) -> None:
        import pptree

        pptree.print_tree(
            current_node=self.root,
            childattr="children",
            nameattr="tree_label",
            horizontal=True,
        )

    def __getitem__(
        self, key: Union[int, slice, str]
    ) -> Union[DirectedTreeNode, List[DirectedTreeNode]]:
        # Get the nodes' dictionary (already inserted in order following BFS)
        nodes_dict = self.nodes_dict

        if isinstance(key, str):
            if key not in nodes_dict.keys():
                raise KeyError(key)

            return nodes_dict[key]

        if isinstance(key, int):
            if key > len(nodes_dict):
                raise IndexError(key)

            return list(nodes_dict.values())[key]

        if isinstance(key, slice):
            return list(nodes_dict.values())[key]

        raise TypeError(type(key).__name__)

    def __len__(self) -> int:
        return len(self.nodes_dict)

    def __iter__(self) -> Iterable[DirectedTreeNode]:
        yield from DirectedTree.breadth_first_search(root=self.root)

    def __reversed__(self) -> Iterable[DirectedTreeNode]:
        yield from reversed(self)

    def __contains__(self, item: Union[str, DirectedTreeNode]) -> bool:
        if isinstance(item, str):
            return item in self.nodes_dict.keys()

        if isinstance(item, DirectedTreeNode):
            return item.name() in self.nodes_dict.keys()

        raise TypeError(type(item).__name__)
