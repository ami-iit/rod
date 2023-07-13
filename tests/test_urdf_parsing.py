import pytest
import robot_descriptions
import robot_descriptions.loaders.idyntree
from utils_models import ModelFactory, Robot

import rod


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
def test_urdf_parsing(robot: Robot) -> None:
    """Test parsing URDF files."""

    # Get the path to the URDF
    urdf_path = ModelFactory.get_model_description(robot=robot)

    # Check that it fails if is_urdf=False and the resource is a path
    with pytest.raises(RuntimeError):
        _ = rod.Sdf.load(sdf=urdf_path, is_urdf=False)

    # Check that it fails if is_urdf=False and the resource is a path string
    with pytest.raises(RuntimeError):
        _ = rod.Sdf.load(sdf=str(urdf_path), is_urdf=False)

    # Check that it fails if is_urdf=False and the resource is an urdf string
    with pytest.raises(RuntimeError):
        _ = rod.Sdf.load(sdf=urdf_path.read_text(), is_urdf=False)

    # Check that it fails if is_urdf=False and the resource is an urdf string
    with pytest.raises(RuntimeError):
        _ = rod.Sdf.load(sdf=urdf_path.read_text(), is_urdf=None)

    # The following instead should succeed
    _ = rod.Sdf.load(sdf=urdf_path, is_urdf=None)
    _ = rod.Sdf.load(sdf=urdf_path, is_urdf=True)
    _ = rod.Sdf.load(sdf=str(urdf_path), is_urdf=None)
    _ = rod.Sdf.load(sdf=str(urdf_path), is_urdf=True)
    _ = rod.Sdf.load(sdf=urdf_path.read_text(), is_urdf=True)

    # Load once again the urdf
    rod_sdf = rod.Sdf.load(sdf=urdf_path, is_urdf=True)

    # There should be only one model
    assert len(rod_sdf.models()) == 1
    assert rod_sdf.models()[0] == rod_sdf.model

    # Note: when a URDF is loaded into ROD, it gets first converted to SDF by sdformat.
    # This pre-processing might alter the URDF, especially if fixed joints are present.

    # Load the model in iDynTree (w/o specifying the joints list)
    idt_model_full = robot_descriptions.loaders.idyntree.load_robot_description(
        description_name=robot.to_module_name(), joints_list=None
    )

    # Check the canonical link
    assert rod_sdf.model.get_canonical_link() == idt_model_full.getLinkName(
        idt_model_full.getDefaultBaseLink()
    )

    # Extract data from the ROD model
    link_names = [l.name for l in rod_sdf.model.links()]
    joint_names = [j.name for j in rod_sdf.model.joints()]
    joint_names_dofs = [j.name for j in rod_sdf.model.joints() if j.type != "fixed"]

    # Remove the world joint if the model is fixed-base
    if rod_sdf.model.is_fixed_base():
        world_joints = [
            j.name
            for j in rod_sdf.model.joints()
            if j.type == "fixed" and j.parent == "world"
        ]
        assert len(world_joints) == 1
        _ = joint_names.pop(joint_names.index(world_joints[0]))
        assert _ == world_joints[0]

    # New sdformat versions, after lumping fixed joints, create a new frame called
    # "<removed_fake_link>_fixed_joint"
    frame_names = [
        f.name for f in rod_sdf.model.frames() if not f.name.endswith("_fixed_joint")
    ]

    # Check the number of DoFs (all non-fixed joints)
    assert len(joint_names_dofs) == idt_model_full.getNrOfDOFs()

    link_names_idt = [
        idt_model_full.getLinkName(idx) for idx in range(idt_model_full.getNrOfLinks())
    ]

    joint_names_idt = [
        idt_model_full.getJointName(idx)
        for idx in range(idt_model_full.getNrOfJoints())
    ]

    # Note: iDynTree also includes sensor frames here, while ROD does not
    frame_names_idt = [
        idt_model_full.getFrameName(idx)
        for idx in range(idt_model_full.getNrOfLinks(), idt_model_full.getNrOfFrames())
    ]

    assert set(link_names_idt) == set(link_names)
    assert set(joint_names_idt) == set(joint_names)
    assert set(frame_names) - set(frame_names_idt) == set()

    total_mass = sum([l.inertial.mass for l in rod_sdf.model.links()])
    assert total_mass == pytest.approx(idt_model_full.getTotalMass())
