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
    model: rod.Model,
    frame_convention: FrameConvention,
    is_top_level: bool = True,
    attach_frames_to_links: bool = True,
) -> None:

    # Resolve all implicit reference frames using Sdf convention
    model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

    # =============================
    # Initialize forward kinematics
    # =============================

    from rod.kinematics.tree_transforms import TreeTransforms

    # Create the object to compute the kinematics of the tree.
    kin = TreeTransforms.build(model=model, is_top_level=is_top_level)

    # =====================================
    # Update frames to be attached to links
    # =====================================

    # Update the //frame/attached_to attribute of all frames so that they are
    # directly attached to links.
    if attach_frames_to_links:
        for frame in model.frames():
            # Find the link to which the frame is attached to following recursively
            # the //frame/attached_to attribute.
            parent_link = find_parent_link_of_frame(frame=frame, model=model)

            # Compute the transform between the model and the frame.
            model_H_frame = (
                kin.relative_transform(
                    relative_to="__model__", name=frame.pose.relative_to
                )
                @ frame.pose.transform()
            )

            # Compute the transform between the parent link and the model.
            parent_link_H_model = kin.relative_transform(
                relative_to=parent_link, name="__model__"
            )

            # Update the frame such that it is attached_to a link, populating the
            # pose with the correct transform between the parent link and the frame.
            frame.attached_to = parent_link
            frame.pose = rod.Pose.from_transform(
                relative_to=parent_link,
                transform=parent_link_H_model @ model_H_frame,
            )

    # =============================================================
    # Define the default reference frames of the different elements
    # =============================================================

    match frame_convention:
        case FrameConvention.World:
            reference_frame_model = lambda m: "world"
            reference_frame_links = lambda l: "world"
            reference_frame_frames = lambda f: "world"
            reference_frame_joints = lambda j: "world"
            reference_frame_visuals = lambda v: "world"
            reference_frame_inertials = lambda i, parent_link: "world"
            reference_frame_collisions = lambda c: "world"
            reference_frame_link_canonical = "world"

        case FrameConvention.Model:

            reference_frame_model = lambda m: "world"
            reference_frame_links = lambda l: "__model__"
            reference_frame_frames = lambda f: "__model__"
            reference_frame_joints = lambda j: "__model__"
            reference_frame_visuals = lambda v: "__model__"
            reference_frame_inertials = lambda i, parent_link: "__model__"
            reference_frame_collisions = lambda c: "__model__"
            reference_frame_link_canonical = "__model__"

        case FrameConvention.Sdf:

            visual_name_to_parent_link = {
                visual_name: parent_link
                for d in [
                    {v.name: link for v in link.visuals()} for link in model.links()
                ]
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
            reference_frame_frames = lambda f: f.attached_to
            reference_frame_joints = lambda j: joint.child
            reference_frame_visuals = lambda v: visual_name_to_parent_link[v.name].name
            reference_frame_inertials = lambda i, parent_link: parent_link.name
            reference_frame_collisions = lambda c: collision_name_to_parent_link[
                c.name
            ].name
            reference_frame_link_canonical = "__model__"

        case FrameConvention.Urdf:

            visual_name_to_parent_link = {
                visual_name: parent_link
                for d in [
                    {v.name: link for v in link.visuals()} for link in model.links()
                ]
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
            reference_frame_frames = lambda f: f.attached_to
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

        case _:
            raise ValueError(frame_convention)

    # =========================================
    # Process the reference frames of the model
    # =========================================

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
        x_H_frame = frame.pose.transform()
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


def find_parent_link_of_frame(frame: rod.Frame, model: rod.Model) -> str:

    links_dict = {l.name: l for l in model.links()}
    frames_dict = {f.name: f for f in model.frames()}
    joints_dict = {j.name: j for j in model.joints()}
    sub_models_dict = {m.name: m for m in model.models()}

    assert isinstance(frame, rod.Frame)

    match frame.attached_to:
        case anchor if anchor in links_dict:
            parent = links_dict[frame.attached_to]

        case anchor if anchor in frames_dict:
            parent = frames_dict[frame.attached_to]

        case frame if frame.attached in {model.name, "__model__"}:
            return model.get_canonical_link()

        case anchor if anchor in joints_dict:
            raise ValueError("Frames cannot be attached to joints")

        case anchor if anchor in sub_models_dict:
            raise RuntimeError("Model composition not yet supported")

        case _:
            raise RuntimeError(
                f"Failed to find element with name '{frame.attached_to}'"
            )

    # At this point, the parent is either a link or another frame.
    assert isinstance(parent, (rod.Link, rod.Frame))

    match parent:
        # If the parent is a link, can stop searching.
        case parent if isinstance(parent, rod.Link):
            return parent.name

        # If the parent is another frame, keep looking for the parent link.
        case parent if isinstance(parent, rod.Frame):
            return find_parent_link_of_frame(frame=parent, model=model)

    raise RuntimeError("This recursive function should never arrive here.")
