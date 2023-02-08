import enum
from collections import defaultdict

import numpy as np

import rod
from rod import logging


class FrameConvention(enum.IntEnum):
    Model = enum.auto()
    Sdf = enum.auto()
    Urdf = enum.auto()
    World = enum.auto()


def switch_frame_convention(
    model: "rod.Model", frame_convention: FrameConvention, is_top_level: bool = True
) -> None:
    # Resolve all implicit reference frames using Sdf convention
    model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

    # =============================================================
    # Define the default reference frames of the different elements
    # =============================================================

    if frame_convention is FrameConvention.World:
        reference_frame_model = lambda m: "world"
        reference_frame_links = lambda l: "world"
        reference_frame_frames = lambda f: "world"
        reference_frame_joints = lambda j: "world"
        reference_frame_visuals = lambda v: "world"
        reference_frame_inertials = lambda i, parent_link: "world"
        reference_frame_collisions = lambda c: "world"
        reference_frame_link_canonical = "world"

    elif frame_convention is FrameConvention.Model:
        reference_frame_model = lambda m: "world"
        reference_frame_links = lambda l: "__model__"
        reference_frame_frames = lambda f: "__model__"
        reference_frame_joints = lambda j: "__model__"
        reference_frame_visuals = lambda v: "__model__"
        reference_frame_inertials = lambda i, parent_link: "__model__"
        reference_frame_collisions = lambda c: "__model__"
        reference_frame_link_canonical = "__model__"

    elif frame_convention is FrameConvention.Sdf:
        visual_name_to_parent_link = {
            visual_name: parent_link
            for d in [{v.name: link for v in link.visuals()} for link in model.links()]
            for visual_name, parent_link in d.items()
        }

        collision_name_to_parent_link = {
            collision_name: parent_link
            for d in [
                {c.name: link for c in link.collisions()} for link in model.links()
            ]
            for collision_name, parent_link in d.items()
        }

        reference_frame_model = lambda m: "world"
        reference_frame_links = lambda l: "__model__"
        reference_frame_frames = lambda f: "__model__"
        reference_frame_joints = lambda j: joint.child
        reference_frame_visuals = lambda v: visual_name_to_parent_link[v.name].name
        reference_frame_inertials = lambda i, parent_link: parent_link.name
        reference_frame_collisions = lambda c: collision_name_to_parent_link[
            c.name
        ].name
        reference_frame_link_canonical = "__model__"

    elif frame_convention is FrameConvention.Urdf:
        visual_name_to_parent_link = {
            visual_name: parent_link
            for d in [{v.name: link for v in link.visuals()} for link in model.links()]
            for visual_name, parent_link in d.items()
        }

        collision_name_to_parent_link = {
            collision_name: parent_link
            for d in [
                {c.name: link for c in link.collisions()} for link in model.links()
            ]
            for collision_name, parent_link in d.items()
        }

        link_name_to_parent_joint_names = defaultdict(list)

        for j in model.joints():
            if j.child != model.get_canonical_link():
                link_name_to_parent_joint_names[j.child].append(j.name)
            else:
                # The pose of the canonical link is used to define the origin of
                # the URDF joint connecting the world to the robot
                assert model.is_fixed_base()
                link_name_to_parent_joint_names[j.child].append("world")

        reference_frame_model = lambda m: "world"
        reference_frame_links = lambda l: link_name_to_parent_joint_names[l.name][0]
        reference_frame_frames = lambda f: "__model__"
        reference_frame_joints = lambda j: j.parent
        reference_frame_visuals = lambda v: visual_name_to_parent_link[v.name].name
        reference_frame_inertials = lambda i, parent_link: parent_link.name
        reference_frame_collisions = lambda c: collision_name_to_parent_link[
            c.name
        ].name

        if model.is_fixed_base():
            canonical_link = {l.name: l for l in model.links()}[
                model.get_canonical_link()
            ]
            reference_frame_link_canonical = reference_frame_links(l=canonical_link)
        else:
            reference_frame_link_canonical = "__model__"
    else:
        raise ValueError(frame_convention)

    # =========================================
    # Process the reference frames of the model
    # =========================================

    from rod.kinematics.tree_transforms import TreeTransforms

    # Create the tree from the model, using the original frame convention
    # (otherwise it would call this helper obtaining an infinite loop)
    kin = TreeTransforms.build(
        model=model, is_top_level=is_top_level, prevent_switching_frame_convention=True
    )

    if is_top_level:
        assert model.pose.relative_to in {"", None}

    else:
        # Adjust the reference frame of the sub-model
        if model.pose.relative_to != reference_frame_model:
            x_H_model = model.pose.transform()
            target_H_x = kin.relative_transform(
                relative_to=reference_frame_model(m=model),
                name=model.pose.relative_to,
            )

            model.pose = rod.Pose.from_transform(
                relative_to=reference_frame_model(m=model),
                transform=target_H_x @ x_H_model,
            )

    # Adjust the reference frames of all sub-models
    for sub_model in model.models():
        logging.info(
            "Model composition not yet supported, ignoring '{}/{}'".format(
                model.name, sub_model.name
            )
        )

    # Adjust the reference frames of all joints
    for joint in model.joints():
        x_H_joint = joint.pose.transform()
        target_H_x = kin.relative_transform(
            relative_to=reference_frame_joints(j=joint),
            name=joint.pose.relative_to,
        )

        joint.pose = rod.Pose.from_transform(
            relative_to=reference_frame_joints(j=joint),
            transform=target_H_x @ x_H_joint,
        )

    # Adjust the reference frames of all frames
    for frame in model.frames():
        x_H_frame = frame.pose.transform() if frame.pose is not None else np.eye(4)

        target_H_x = kin.relative_transform(
            relative_to=reference_frame_frames(f=frame),
            name=frame.pose.relative_to,
        )

        frame.pose = rod.Pose.from_transform(
            relative_to=reference_frame_frames(f=frame),
            transform=target_H_x @ x_H_frame,
        )

    # Adjust the reference frames of all links
    for link in model.links():
        relative_to = (
            reference_frame_links(l=link)
            if link.name != model.get_canonical_link()
            else reference_frame_link_canonical
        )

        # Link pose
        x_H_link = link.pose.transform()
        target_H_x = kin.relative_transform(
            relative_to=relative_to,
            name=link.pose.relative_to,
        )
        link.pose = rod.Pose.from_transform(
            relative_to=relative_to,
            transform=target_H_x @ x_H_link,
        )

        # Inertial pose
        x_H_inertial = link.inertial.pose.transform()
        target_H_x = kin.relative_transform(
            relative_to=reference_frame_inertials(i=link.inertial, parent_link=link),
            name=link.inertial.pose.relative_to,
        )
        link.inertial.pose = rod.Pose.from_transform(
            relative_to=reference_frame_inertials(i=link.inertial, parent_link=link),
            transform=target_H_x @ x_H_inertial,
        )

        # Visuals pose
        for visual in link.visuals():
            x_H_visual = visual.pose.transform()
            target_H_x = kin.relative_transform(
                relative_to=reference_frame_visuals(v=visual),
                name=visual.pose.relative_to,
            )

            visual.pose = rod.Pose.from_transform(
                relative_to=reference_frame_visuals(v=visual),
                transform=target_H_x @ x_H_visual,
            )

        # Collisions pose
        for collision in link.collisions():
            x_H_collision = collision.pose.transform()
            target_H_x = kin.relative_transform(
                relative_to=reference_frame_collisions(c=collision),
                name=collision.pose.relative_to,
            )

            collision.pose = rod.Pose.from_transform(
                relative_to=reference_frame_collisions(c=collision),
                transform=target_H_x @ x_H_collision,
            )
