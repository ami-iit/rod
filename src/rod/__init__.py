from . import logging
from .sdf.collision import Collision
from .sdf.common import Frame, Pose, Xyz
from .sdf.geometry import (
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
from .sdf.joint import Axis, Dynamics, Joint, Limit
from .sdf.link import Inertia, Inertial, Link
from .sdf.material import Material
from .sdf.model import Model
from .sdf.physics import Physics
from .sdf.scene import Scene
from .sdf.sdf import Sdf
from .sdf.visual import Visual
from .sdf.world import World
from .utils.frame_convention import FrameConvention


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
