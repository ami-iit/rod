from . import logging
from .collision import Collision
from .common import Frame, Pose, Xyz
from .geometry import (
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    Geometry,
    Heightmap,
    Mesh,
    Plane,
    Sphere,
)
from .joint import Axis, Dynamics, Joint, Limit
from .link import Inertia, Inertial, Link
from .material import Material
from .model import Model
from .physics import Physics
from .scene import Scene
from .sdf import Sdf
from .visual import Visual
from .world import World


def _is_editable():

    import importlib.util
    import pathlib
    import site

    # Get the ModuleSpec of rod
    rod_spec = importlib.util.find_spec(name="rod")

    # This can be None. If it's None, assume non-editable installation.
    if rod_spec.origin is None:
        return False

    # Get the folder containing the rod package
    rod_package_dir = str(pathlib.Path(rod_spec.origin).parent.parent)

    # The installation is editable if the package dir is not in any {site|dist}-packages
    return rod_package_dir not in site.getsitepackages()


# Initialize the logging verbosity
logging.configure(
    level=logging.LoggingLevel.DEBUG if _is_editable() else logging.LoggingLevel.WARNING
)

del _is_editable
