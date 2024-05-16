import idyntree.bindings as idt
import numpy as np
import numpy.typing as npt
import pytest
from utils_models import ModelFactory, Robot

import rod
import rod.urdf.exporter


@pytest.mark.parametrize(
    "robot",
    [
        Robot.iCub,
        Robot.DoublePendulum,
        Robot.Cassie,
        Robot.Ur10,
        Robot.AtlasV4,
        Robot.Ergocub,
    ],
)
def test_urdf_exporter(robot: Robot) -> None:
    """Test exporting URDF files."""

    # Get the path to the URDF.
    original_urdf_path = ModelFactory.get_model_description(robot=robot)

    # Load the URDF (it gets converted to SDF internally).
    sdf = rod.Sdf.load(sdf=original_urdf_path, is_urdf=True)

    # Export the URDF from the in-memory SDF-based description.
    exported_urdf_string = rod.urdf.exporter.UrdfExporter().to_urdf_string(sdf=sdf)

    # Create two model loaders.
    mdl_loader_original = idt.ModelLoader()
    mdl_loader_exported = idt.ModelLoader()

    # Get the joint serialization from ROD.
    joint_names = [j.name for j in sdf.model.joints() if j.type != "fixed"]

    # Load the original URDF.
    assert mdl_loader_original.loadReducedModelFromString(
        original_urdf_path.read_text(), joint_names
    )

    # Load the exported URDF.
    assert mdl_loader_exported.loadReducedModelFromString(
        exported_urdf_string, joint_names
    )

    # Create two KinDynComputations objects, one for the original URDF and
    # one for the exported URDF.
    kin_dyn_original = idt.KinDynComputations()
    kin_dyn_exported = idt.KinDynComputations()

    # Load the two robot models in the KinDynComputations objects.
    assert kin_dyn_original.loadRobotModel(mdl_loader_original.model())
    assert kin_dyn_exported.loadRobotModel(mdl_loader_exported.model())

    # Get all the links and frames from the original URDF.
    all_original_frames = [
        kin_dyn_original.getFrameName(i)
        for i in range(
            kin_dyn_original.getNrOfLinks(), kin_dyn_original.getNrOfFrames()
        )
    ]

    all_exported_frames = [
        kin_dyn_exported.getFrameName(i)
        for i in range(
            kin_dyn_exported.getNrOfLinks(), kin_dyn_exported.getNrOfFrames()
        )
    ]

    # =================================
    # Test that kinematics is preserved
    # =================================

    def get_frame_transform(
        kin_dyn: idt.KinDynComputations, frame_name: str
    ) -> npt.NDArray:

        frame_idx = kin_dyn.getFrameIndex(frame_name)
        assert frame_idx >= 0, frame_name

        if frame_name == kin_dyn.getFloatingBase():
            H_idt = kin_dyn.getWorldBaseTransform()
        else:
            H_idt = kin_dyn.getWorldTransform(frame_name)

        H = np.eye(4)
        H[0:3, 3] = H_idt.getPosition().toNumPy()
        H[0:3, 0:3] = H_idt.getRotation().toNumPy()

        return H

    for frame in set(all_original_frames).intersection(all_exported_frames):

        W_H_F_original = get_frame_transform(kin_dyn=kin_dyn_original, frame_name=frame)
        W_H_F_exported = get_frame_transform(kin_dyn=kin_dyn_exported, frame_name=frame)

        assert W_H_F_exported == pytest.approx(W_H_F_original, abs=1e-6), (
            frame,
            W_H_F_original,
            W_H_F_exported,
            np.abs(W_H_F_original - W_H_F_exported),
        )

    # ===============================
    # Test that dynamics is preserved
    # ===============================

    mass_original = kin_dyn_original.model().getTotalMass()
    mass_exported = kin_dyn_exported.model().getTotalMass()
    assert mass_exported == pytest.approx(mass_original, abs=1e-6)

    locked_inertia_original = (
        kin_dyn_original.getRobotLockedInertia().asMatrix().toNumPy()
    )

    locked_inertia_exported = (
        kin_dyn_exported.getRobotLockedInertia().asMatrix().toNumPy()
    )

    assert locked_inertia_exported == pytest.approx(locked_inertia_original, abs=1e-6)
