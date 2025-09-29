import enum

import rod
from rod.kinematics.tree_transforms import TreeTransforms


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
    """Switch the frame convention of a model."""

    # Resolve all implicit reference frames
    model.resolve_frames(is_top_level=is_top_level, explicit_frames=True)

    # Initialize kinematics
    kin = TreeTransforms.build(model=model, is_top_level=is_top_level)

    # Attach frames to links if requested
    if attach_frames_to_links:
        _attach_frames_to_links(model, kin)

    # Get frame mapping functions
    frame_fn = _get_frame_function(frame_convention, model)

    # Process all elements
    _process_model_elements(model, kin, frame_fn, is_top_level)


def _attach_frames_to_links(model: rod.Model, kin) -> None:
    """Attach all frames directly to links."""
    for frame in model.frames():
        parent_link = find_parent_link_of_frame(frame, model)

        model_H_frame = (
            kin.relative_transform(
                from_frame="__model__", to_frame=frame.pose.relative_to
            )
            @ frame.pose.transform()
        )
        parent_link_H_model = kin.relative_transform(
            from_frame=parent_link, to_frame="__model__"
        )

        frame.attached_to = parent_link
        frame.pose = rod.Pose.from_transform(
            relative_to=parent_link,
            transform=parent_link_H_model @ model_H_frame,
        )


def _get_frame_function(convention: FrameConvention, model: rod.Model):
    """Get frame mapping function for the convention."""

    # Pre-compute mappings for SDF and URDF
    if convention in (FrameConvention.Sdf, FrameConvention.Urdf):
        visual_map = {
            v.name: link.name for link in model.links() for v in link.visuals()
        }
        collision_map = {
            c.name: link.name for link in model.links() for c in link.collisions()
        }

        if convention == FrameConvention.Urdf:
            joint_map = {}
            canonical = model.get_canonical_link()
            for joint in model.joints():
                joint_map[joint.child] = (
                    "world"
                    if joint.child == canonical and model.is_fixed_base()
                    else joint.name
                )
            canonical_ref = joint_map.get(canonical, "__model__")

    def get_target_frame(element_type: str, element, parent_link=None):
        dispatch = {
            FrameConvention.World: {
                None: "world",
            },
            FrameConvention.Model: {
                "model": "world",
                None: "__model__",
            },
            FrameConvention.Sdf: {
                "model": "world",
                "link": "__model__",
                "frame": lambda e: e.attached_to,
                "joint": lambda e: e.child,
                "visual": lambda e: visual_map[e.name],
                "collision": lambda e: collision_map[e.name],
                "inertial": lambda e: parent_link.name,
                "canonical": "__model__",
            },
            FrameConvention.Urdf: {
                "model": "world",
                "link": lambda e: joint_map[e.name],
                "frame": lambda e: e.attached_to,
                "joint": lambda e: e.parent,
                "visual": lambda e: visual_map[e.name],
                "collision": lambda e: collision_map[e.name],
                "inertial": lambda e: parent_link.name,
                "canonical": canonical_ref,
            },
        }

        table = dispatch[convention]
        value = table.get(element_type, table.get(None))

        return value(element) if callable(value) else value

    return get_target_frame


def _transform_pose(kin, pose, target_frame: str):
    """Transform a pose to target frame."""
    target_H_current = kin.relative_transform(
        from_frame=target_frame, to_frame=pose.relative_to
    )
    return rod.Pose.from_transform(
        relative_to=target_frame,
        transform=target_H_current @ pose.transform(),
    )


def _process_model_elements(
    model: rod.Model, kin, frame_fn, is_top_level: bool
) -> None:
    """Process all model elements with the frame function."""
    canonical = model.get_canonical_link()

    # Model pose for sub-models
    if not is_top_level:
        target = frame_fn("model", model)
        if model.pose.relative_to != target:
            model.pose = _transform_pose(kin, model.pose, target)

    # Process all elements
    for joint in model.joints():
        joint.pose = _transform_pose(kin, joint.pose, frame_fn("joint", joint))

    for frame in model.frames():
        frame.pose = _transform_pose(kin, frame.pose, frame_fn("frame", frame))

    for link in model.links():
        # Link pose
        target = (
            frame_fn("canonical", link)
            if link.name == canonical
            else frame_fn("link", link)
        )
        link.pose = _transform_pose(kin, link.pose, target)

        # Link elements
        link.inertial.pose = _transform_pose(
            kin, link.inertial.pose, frame_fn("inertial", link.inertial, link)
        )

        for visual in link.visuals():
            visual.pose = _transform_pose(kin, visual.pose, frame_fn("visual", visual))

        for collision in link.collisions():
            collision.pose = _transform_pose(
                kin, collision.pose, frame_fn("collision", collision)
            )


def find_parent_link_of_frame(frame: rod.Frame, model: rod.Model) -> str:
    """Find the parent link of a frame."""
    # Create lookup dicts
    elements = {
        **{l.name: l for l in model.links()},
        **{f.name: f for f in model.frames()},
        **{j.name: j for j in model.joints()},
        **{m.name: m for m in model.models()},
    }

    # Handle special cases
    if frame.attached_to in {model.name, "__model__"}:
        return model.get_canonical_link()

    if frame.attached_to not in elements:
        raise RuntimeError(f"Element '{frame.attached_to}' not found")

    parent = elements[frame.attached_to]

    # Check parent type
    if isinstance(parent, rod.Link):
        return parent.name
    elif isinstance(parent, rod.Frame):
        return find_parent_link_of_frame(parent, model)  # Recursive
    elif isinstance(parent, rod.Joint):
        raise ValueError("Frames cannot be attached to joints")
    else:
        raise RuntimeError("Model composition not yet supported")
