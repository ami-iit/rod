import tempfile

import numpy as np
import pytest
import trimesh

from rod.builder.primitives import MeshBuilder


def test_builder_creation():

    # Create a mesh of a box primitive.
    mesh = trimesh.creation.box([1, 1, 1])

    # Temporary write to file because rod Mesh works with uri.
    with tempfile.TemporaryDirectory() as tmp:
        with tempfile.NamedTemporaryFile(suffix=".stl", dir=tmp, delete=False) as fp:

            mesh.export(fp.name, file_type="stl")
            fp.close()

            builder = MeshBuilder(
                name="test_mesh",
                mesh_uri=fp.name,
                mass=1.0,
                scale=np.array([1.0, 1.0, 1.0]),
            )

    # Check that the builder can build a correct link.
    # Note that the URI is not valid since it's a temporary file.
    link = builder.build_link().add_inertial().add_visual().add_collision().build()
    assert link.collision is not None
    assert link.collision.geometry.mesh is not None
    assert link.collision.geometry.mesh.scale == pytest.approx([1, 1, 1])

    assert (
        builder.mesh.vertices.shape == mesh.vertices.shape
    ), f"{builder.mesh.vertices.shape} != {mesh.vertices.shape}"

    assert (
        builder.mesh.faces.shape == mesh.faces.shape
    ), f"{builder.mesh.faces.shape} != {mesh.faces.shape}"

    assert (
        builder.mesh.moment_inertia.all() == mesh.moment_inertia.all()
    ), f"{builder.mesh.moment_inertia} != {mesh.moment_inertia}"

    assert builder.mesh.volume == mesh.volume, f"{builder.mesh.volume} != {mesh.volume}"


def test_builder_creation_custom_mesh():

    # Create a custom mesh
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

    # Temporary write to file because rod Mesh works with uri.
    with tempfile.TemporaryDirectory() as tmp:
        with tempfile.NamedTemporaryFile(suffix=".stl", dir=tmp, delete=False) as fp:

            mesh.export(fp.name, file_type="stl")
            fp.close()

            builder = MeshBuilder(
                name="test_mesh",
                mesh_uri=fp.name,
                mass=1.0,
                scale=np.array([1.0, 1.0, 1.0]),
            )

    assert (
        builder.mesh.vertices.shape == mesh.vertices.shape
    ), f"{builder.mesh.vertices.shape} != {mesh.vertices.shape}"

    assert (
        builder.mesh.faces.shape == mesh.faces.shape
    ), f"{builder.mesh.faces.shape} != {mesh.faces.shape}"

    assert (
        builder.mesh.moment_inertia.all() == mesh.moment_inertia.all()
    ), f"{builder.mesh.moment_inertia} != {mesh.moment_inertia}"

    assert builder.mesh.volume == mesh.volume, f"{builder.mesh.volume} != {mesh.volume}"
