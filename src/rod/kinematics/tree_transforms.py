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

    @staticmethod
    def build(
        model: "rod.Model",
        is_top_level: bool = True,
        prevent_switching_frame_convention: bool = False,
    ) -> "TreeTransforms":
        model = copy.deepcopy(model)

        model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

        if not prevent_switching_frame_convention:
            model.switch_frame_convention(frame_convention=rod.FrameConvention.Urdf)

        return TreeTransforms(
            kinematic_tree=KinematicTree.build(model=model, is_top_level=is_top_level)
        )

    def transform(self, name: str) -> npt.NDArray:
        if name == TreeFrame.WORLD:
            return np.eye(4)

        if name in {TreeFrame.MODEL, self.kinematic_tree.model.name}:
            relative_to = self.kinematic_tree.model.pose.relative_to
            assert relative_to in {None, ""}, (relative_to, name)
            return self.kinematic_tree.model.pose.transform()

        if name in self.kinematic_tree.joint_names():
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

        if (
            name in self.kinematic_tree.link_names()
            or name in self.kinematic_tree.frame_names()
        ):
            element = (
                self.kinematic_tree.links_dict[name]
                if name in self.kinematic_tree.link_names()
                else self.kinematic_tree.frames_dict[name]
            )
            assert element.name() == name

            # Get the pose of the frame in which the node's pose is expressed
            assert element._source.pose.relative_to not in {"", None}
            x_H_N = element._source.pose.transform()
            W_H_x = self.transform(name=element._source.pose.relative_to)

            # Compute and cache the world-to-node transform
            W_H_N = W_H_x @ x_H_N

            return W_H_N

        raise ValueError(name)

    def relative_transform(self, relative_to: str, name: str) -> npt.NDArray:
        return np.linalg.inv(self.transform(name=relative_to)) @ self.transform(
            name=name
        )
