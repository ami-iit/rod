import dataclasses
import os
import pathlib
import shutil
import subprocess
import tempfile
from typing import List, Optional, Union

import mashumaro
import packaging.specifiers
import packaging.version
import xmltodict

from .element import Element
from .model import Model
from .world import World


@dataclasses.dataclass
class Sdf(Element):

    version: str = dataclasses.field(metadata=mashumaro.field_options(alias="@version"))

    world: Optional[Union[World, List[World]]] = dataclasses.field(default=None)

    model: Optional[Union[Model, List[Model]]] = dataclasses.field(default=None)

    @staticmethod
    def load(sdf: Union[pathlib.Path, str], is_urdf: Optional[bool] = None) -> "Sdf":
        """
        Load an SDF resource.

        The SDF resource could either be a `pathlibPath` or a `str` containing
        either the path to the SDF file or an already parsed SDF string.

        Args:
            sdf: The SDF resource to load.
            is_urdf: Marks the SDF resource as an URDF to convert.

        Returns:
            The parsed SDF file.
        """

        # Handle the max path length depending on the OS
        try:
            from ctypes.wintypes import MAX_PATH
        except ValueError:
            MAX_PATH = os.pathconf("/", "PC_PATH_MAX")

        # Get the SDF/URDF string from the input resource.
        # If is_urdf is None and the input resource is a file path, we try to guess
        # the is_urdf flag checking the file extension.

        # Check first if it's a Path object
        if isinstance(sdf, pathlib.Path):
            sdf_string = sdf.read_text()
            is_urdf = is_urdf if is_urdf is not None else sdf.suffix == ".urdf"

        # Then, check if it's a string with a path
        elif (
            isinstance(sdf, str)
            and len(sdf) <= MAX_PATH
            and pathlib.Path(sdf).is_file()
        ):
            sdf_string = pathlib.Path(sdf).read_text()
            is_urdf = (
                is_urdf if is_urdf is not None else pathlib.Path(sdf).suffix == ".urdf"
            )

        # Finally, it must be a SDF/URDF string
        else:
            sdf_string = sdf

        # Convert SDF to URDF if needed (it requires system executables)
        if is_urdf:
            urdf_string = sdf_string
            sdf_string = Sdf._urdf_string_to_sdf_string(
                urdf_string=urdf_string, gazebo_executable=Sdf._get_gazebo_executable()
            )

        # Parse the SDF to dict
        try:
            xml_dict = xmltodict.parse(xml_input=sdf_string)
        except Exception:
            raise ValueError("Failed to parse 'sdf' argument")

        # Look for the top-level <sdf> element
        try:
            sdf_dict = xml_dict["sdf"]
        except KeyError:
            raise RuntimeError("Failed to find top-level '<sdf>' element")

        # Get the SDF version
        sdf = Sdf.from_dict(sdf_dict)
        sdf_version = packaging.version.Version(sdf_dict["@version"])

        # Check that the SDF version is compatible
        if sdf_version not in packaging.specifiers.SpecifierSet(">= 1.7"):
            raise RuntimeError(f"Unsupported SDF version: {sdf_version}")

        return sdf

    def serialize(self, pretty: bool = False, indent: str = "  "):

        return xmltodict.unparse(
            input_dict=dict(sdf=self.to_dict()), pretty=pretty, indent=indent
        )

    @staticmethod
    def _urdf_string_to_sdf_string(
        urdf_string: str, gazebo_executable: pathlib.Path
    ) -> str:

        with tempfile.NamedTemporaryFile(mode="w+") as fp:

            fp.write(urdf_string)
            fp.seek(0)

            cp = subprocess.run(
                [str(gazebo_executable), "sdf", "-p", fp.name],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )

        if cp.returncode != 0:
            print(cp.stdout)
            raise RuntimeError("Failed to convert URDF to SDF")

        return cp.stdout

    @staticmethod
    def _get_gazebo_executable() -> pathlib.Path:

        gz = shutil.which("gz")
        ign = shutil.which("ign")

        # Check if either Gazebo Sim or Ignition Gazebo are installed
        if gz is not None:
            executable = pathlib.Path(shutil.which("gz"))
        elif ign is not None:
            executable = pathlib.Path(shutil.which("ign"))
        else:
            msg = "Failed to find either the 'gz' or 'ign' executables in PATH"
            raise FileNotFoundError(msg)

        assert executable.is_file()

        # Check if the sdf plugin of the simulator is installed
        cp = subprocess.run([executable, "sdf", "--help"], capture_output=True)

        if cp.returncode != 0:
            msg = f"Failed to find 'sdf' command part of {executable} installation"
            raise RuntimeError(msg)

        return executable
