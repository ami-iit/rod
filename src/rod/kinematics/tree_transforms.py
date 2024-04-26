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
    kinematic_tree: KinematicTree = dataclasses.dataclass(init=False)
    _transform_cache: dict[str, npt.NDArray] = dataclasses.field(default_factory=dict)

    @staticmethod
    def build(
        model: rod.Model,
        is_top_level: bool = True,
    ) -> TreeTransforms:
        model = copy.deepcopy(model)

        # Make sure that all elements have a pose attribute with explicit 'relative_to'.
        model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

        # Build the kinematic tree and return the TreeTransforms object.
        return TreeTransforms(
            kinematic_tree=KinematicTree.build(model=model, is_top_level=is_top_level)
        )

    def transform(self, name: str) -> npt.NDArray:
        if name in self._transform_cache:
            return self._transform_cache[name]

        self._transform_cache[name] = self._compute_transform(name=name)
        return self._transform_cache[name]

    def _compute_transform(self, name: str) -> npt.NDArray:
        match name:
            case TreeFrame.WORLD:

                return np.eye(4)

            case name if name in {TreeFrame.MODEL, self.kinematic_tree.model.name}:

                relative_to = self.kinematic_tree.model.pose.relative_to
                assert relative_to in {None, ""}, (relative_to, name)
                return self.kinematic_tree.model.pose.transform()

            case name if name in self.kinematic_tree.joint_names():

                edge = self.kinematic_tree.joints_dict[name]
                assert edge.name() == name

                # Get the pose of the frame in which the node's pose is expressed
                assert edge._source.pose.relative_to not in {"", None}
                x_H_E = edge._source.pose.transform()
                W_H_x = self.transform(name=edge._source.pose.relative_to)

                # Compute the world-to-node transform
                # TODO: this assumes all joint positions to be 0
                W_H_E = W_H_x @ x_H_E

                return W_H_E

            case name if name in self.kinematic_tree.link_names():

                element = self.kinematic_tree.links_dict[name]

                assert element.name() == name
                assert element._source.pose.relative_to not in {"", None}

                # Get the pose of the frame in which the link's pose is expressed.
                x_H_L = element._source.pose.transform()
                W_H_x = self.transform(name=element._source.pose.relative_to)

                # Compute the world transform of the link.
                W_H_L = W_H_x @ x_H_L
                return W_H_L

            case name if name in self.kinematic_tree.frame_names():

                element = self.kinematic_tree.frames_dict[name]

                assert element.name() == name
                assert element._source.pose.relative_to not in {"", None}

                # Get the pose of the frame in which the frame's pose is expressed.
                x_H_F = element._source.pose.transform()
                W_H_x = self.transform(name=element._source.pose.relative_to)

                # Compute the world transform of the frame.
                W_H_F = W_H_x @ x_H_F
                return W_H_F

            case _:
                raise ValueError(name)

    def relative_transform(self, relative_to: str, name: str) -> npt.NDArray:

        world_H_name = self.transform(name=name)
        world_H_relative_to = self.transform(name=relative_to)

        return TreeTransforms.inverse(world_H_relative_to) @ world_H_name

    @staticmethod
    def inverse(transform: npt.NDArray) -> npt.NDArray:

        R = transform[0:3, 0:3]
        p = np.vstack(transform[0:3, 3])

        return np.block(
            [
                [R.T, -R.T @ p],
                [0, 0, 0, 1],
            ]
        )
