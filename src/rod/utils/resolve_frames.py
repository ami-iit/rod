import functools
from typing import List, Union

import numpy as np

import rod
from rod.sdf.element import Element


def update_element_with_pose(
    element: Element, default_relative_to: Union[str, List[str]], explicit_frames: bool
) -> None:
    if not hasattr(element, "pose"):
        raise ValueError("The input element has no 'pose' attribute")

    # If there are multiple defaults to detect (e.g. __model__ and <model_name>),
    # we select the first entry of the list as real default
    default_relative_to = (
        [default_relative_to]
        if isinstance(default_relative_to, str)
        else default_relative_to
    )

    if len(default_relative_to) < 1:
        raise ValueError(default_relative_to)

    # Either add a new pose if there is any, or specify the reference frame if missing
    if explicit_frames:
        # Add trivial pose
        if element.pose is None:
            element.pose = rod.Pose(
                pose=list(np.zeros(6)), relative_to=default_relative_to[0]
            )

        # Explicitly define reference frame
        else:
            if element.pose.relative_to in {"", None}:
                element.pose.relative_to = default_relative_to[0]

    # Remove the reference frame if the element has a pose wrt the default choice
    else:
        if element.pose is not None and element.pose.relative_to in {"", None}.union(
            default_relative_to
        ):
            # Remove trivial pose
            if np.allclose(element.pose.pose, np.zeros(6)):
                element.pose = None

            # Remove implicit reference frame
            else:
                element.pose.relative_to = ""


def resolve_model_frames(
    model: "rod.Model", is_top_level: bool = True, explicit_frames: bool = True
) -> None:
    # Close the helper for compactness
    update_element = functools.partial(
        update_element_with_pose, explicit_frames=explicit_frames
    )

    # Update the model
    if is_top_level and explicit_frames:
        if model.pose is None:
            model.pose = rod.Pose(pose=list(np.zeros(6)))
        else:
            assert model.pose.relative_to in {"", None}
    else:
        update_element(element=model, default_relative_to="world")

    for frame in model.frames():
        update_element(element=frame, default_relative_to=["__model__", model.name])

    # Update the links and its children elements
    for link in model.links():
        update_element(element=link, default_relative_to=["__model__", model.name])

        update_element(element=link.inertial, default_relative_to=link.name)

        for visual in link.visuals():
            update_element(element=visual, default_relative_to=link.name)

        for collision in link.collisions():
            update_element(element=collision, default_relative_to=link.name)

    # Update the joints
    for joint in model.joints():
        update_element(element=joint, default_relative_to=joint.child)

    # Update the sub-models
    for sub_model in model.models():
        resolve_model_frames(
            model=sub_model, is_top_level=False, explicit_frames=explicit_frames
        )
