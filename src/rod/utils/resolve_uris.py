import os
import pathlib
from typing import List

from rod import Geometry, logging


def resolve_local_uri(uri: str) -> pathlib.Path:
    # Remove the prefix of the URI
    uri_no_prefix = uri.split(sep="//")[-1]

    paths = []
    paths += paths_from_environment_variable("GZ_SIM_RESOURCE_PATH")
    paths += paths_from_environment_variable("IGN_GAZEBO_RESOURCE_PATH")
    paths += paths_from_environment_variable("GAZEBO_MODEL_PATH")

    # Remove possible duplicates
    paths = list(set(paths))

    for path in paths:
        tentative = pathlib.Path(path) / uri_no_prefix

        if tentative.is_file():
            logging.debug(f"Resolved URI: '{tentative}'")
            return tentative

    raise RuntimeError(f"Failed to resolve URI: {uri}")


def resolve_geometry_uris(geometry: Geometry) -> None:
    # Resolve only mesh geometries
    if geometry.mesh is None:
        return

    geometry.mesh.uri = str(resolve_local_uri(uri=geometry.mesh.uri))


def paths_from_environment_variable(variable_name: str) -> List[str]:
    if variable_name not in os.environ:
        return []

    # Collect all paths removing the empty ones (if '::' is part of the variable)
    paths = [p for p in os.environ[variable_name].split(":") if p != ""]

    # Remove duplicates that might occur
    paths = list(set(paths))

    return paths
