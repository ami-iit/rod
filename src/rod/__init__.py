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

# =====================================
# Check for compatible sdformat version
# =====================================


def check_compatible_sdformat(specification_version: str) -> None:
    """
    Check if the installed sdformat version produces SDF files compatible with ROD.

    Args:
        specification_version: The minimum required SDF specification version.

    Note:
        This check runs only if sdformat is installed in the system.
    """

    import os

    import packaging.version
    import xmltodict

    from rod.utils.gazebo import GazeboHelper

    if os.environ.get("ROD_SKIP_SDFORMAT_CHECK", "0") == "1":
        return

    if not GazeboHelper.has_gazebo():
        return
    else:
        cmdline = GazeboHelper.get_gazebo_executable()
        logging.info(f"Calling sdformat through '{cmdline} sdf'")

    output_sdf_version = packaging.version.Version(
        xmltodict.parse(
            xml_input=GazeboHelper.process_model_description_with_sdformat(
                model_description="<sdf version='1.4'/>"
            )
        )["sdf"]["@version"]
    )

    if output_sdf_version < packaging.version.Version(specification_version):
        msg = "The found sdformat installation only supports the '{}' specification, "
        msg += "while ROD requires at least the '{}' specification."
        raise RuntimeError(msg.format(output_sdf_version, specification_version))


check_compatible_sdformat(specification_version="1.10")
del check_compatible_sdformat
