import os
import pathlib

from rod import Geometry, logging


def resolve_local_uri(uri: str) -> pathlib.Path:
    # Remove the prefix of the URI
    uri_no_prefix = uri.split(sep="//")[-1]

    for path in os.environ["IGN_GAZEBO_RESOURCE_PATH"].split(":"):
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
