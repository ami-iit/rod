from __future__ import annotations

import copy
import dataclasses

import numpy as np
import numpy.typing as npt

import rod
from rod.kinematics.kinematic_tree import KinematicTree
from rod.tree import TreeFrame


@dataclasses.dataclass
class TreeTransforms:

    kinematic_tree: KinematicTree
    _cache: dict[str, npt.NDArray] = dataclasses.field(default_factory=dict, init=False)

    @classmethod
    def from_model(cls, model: rod.Model, is_top_level: bool = True) -> TreeTransforms:
        return cls(KinematicTree.build(model=model, is_top_level=is_top_level))

    @staticmethod
    def build(model: rod.Model, is_top_level: bool = True) -> TreeTransforms:
        model = copy.deepcopy(model)
        model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

        kinematic_tree = KinematicTree.build(model=model, is_top_level=is_top_level)

        # Build the kinematic tree and return the TreeTransforms object.
        return TreeTransforms(kinematic_tree=kinematic_tree)

    def transform(self, name: str) -> npt.NDArray:
        """Get world transform, computing and caching path to root as needed."""
        if name in self._cache:
            return self._cache[name]

        # Build path from name to root, stopping at first cached element
        path = []
        current = name
        while current and current not in self._cache:
            path.append(current)
            current = self._get_parent(current)

        # Compute transforms from root down
        base_transform = self._cache.get(current, np.eye(4))
        for element in reversed(path):
            base_transform = base_transform @ self._get_local_transform(element)
            self._cache[element] = base_transform

        return self._cache[name]

    def relative_transform(self, from_frame: str, to_frame: str) -> npt.NDArray:
        """Transform from one frame to another."""
        return self.inverse(self.transform(from_frame)) @ self.transform(to_frame)

    def invalidate(self, name: str) -> None:
        """Remove cached transform and all dependents."""
        to_remove = {key for key in self._cache if self._depends_on(key, name)}
        for key in to_remove:
            del self._cache[key]

    def clear_cache(self) -> None:
        self._cache.clear()

    def _get_parent(self, name: str) -> str | None:
        """Get parent frame for any element."""
        if name == TreeFrame.WORLD:
            return None

        if name in {TreeFrame.MODEL, self.kinematic_tree.model.name}:
            parent = self.kinematic_tree.model.pose.relative_to
            return TreeFrame.WORLD if parent in {None, ""} else parent

        # Search through all element types
        for element_dict in [
            self.kinematic_tree.joints_dict,
            self.kinematic_tree.links_dict,
            self.kinematic_tree.frames_dict,
        ]:
            if name in element_dict:
                parent = element_dict[name]._source.pose.relative_to
                return TreeFrame.WORLD if parent in {"", None} else parent

        raise ValueError(f"Unknown element: {name}")

    def _get_local_transform(self, name: str) -> npt.NDArray:
        """Get local transform for any element."""
        if name == TreeFrame.WORLD:
            return np.eye(4)

        if name in {TreeFrame.MODEL, self.kinematic_tree.model.name}:
            return self.kinematic_tree.model.pose.transform()

        # Search through all element types
        for element_dict in [
            self.kinematic_tree.joints_dict,
            self.kinematic_tree.links_dict,
            self.kinematic_tree.frames_dict,
        ]:
            if name in element_dict:
                return element_dict[name]._source.pose.transform()

        raise ValueError(f"Unknown element: {name}")

    def _depends_on(self, child: str, ancestor: str) -> bool:
        """Check if child depends on ancestor in transform chain."""
        current = child
        while current and current != ancestor:
            current = self._get_parent(current)
        return current == ancestor

    @staticmethod
    def inverse(T: npt.NDArray) -> npt.NDArray:
        R, p = T[:3, :3], T[:3, 3:4]
        return np.block([[R.T, -R.T @ p], [np.zeros((1, 3)), np.ones((1, 1))]])
