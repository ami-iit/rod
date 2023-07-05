import os
import pathlib
import shutil
import subprocess
import tempfile
from typing import Union


class GazeboHelper:
    @staticmethod
    def get_gazebo_executable() -> pathlib.Path:
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

        if not executable.is_file():
            raise TypeError(executable)

        # Check if the sdf plugin of the simulator is installed
        cp = subprocess.run([executable, "sdf", "--help"], capture_output=True)

        if cp.returncode != 0:
            msg = f"Failed to find 'sdf' command part of {executable} installation"
            raise RuntimeError(msg)

        return executable

    @staticmethod
    def has_gazebo() -> bool:
        try:
            _ = GazeboHelper.get_gazebo_executable()
            return True
        except:
            return False

    @staticmethod
    def process_model_description_with_sdformat(
        model_description: Union[str, pathlib.Path]
    ) -> str:
        # =============================
        # Select the correct input type
        # =============================

        # Handle the max path length depending on the OS
        try:
            from ctypes.wintypes import MAX_PATH
        except ValueError:
            MAX_PATH = os.pathconf("/", "PC_PATH_MAX")

        # Check first if it's a Path object
        if isinstance(model_description, pathlib.Path):
            model_description_string = model_description.read_text()

        # Then, check if it's a string with a path
        elif (
            isinstance(model_description, str)
            and len(model_description) <= MAX_PATH
            and pathlib.Path(model_description).is_file()
        ):
            model_description_string = pathlib.Path(model_description).read_text()

        # Finally, it must be a SDF/URDF string
        else:
            model_description_string = model_description

        # ================================
        # Process the string with sdformat
        # ================================

        # Get the Gazebo Sim executable (raises exception if not found)
        gazebo_executable = GazeboHelper.get_gazebo_executable()

        with tempfile.NamedTemporaryFile(mode="w+") as fp:
            fp.write(model_description_string)
            fp.seek(0)

            cp = subprocess.run(
                [str(gazebo_executable), "sdf", "-p", fp.name],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )

        if cp.returncode != 0:
            print(cp.stdout)
            raise RuntimeError("Failed to process the input with sdformat")

        # Get the resulting SDF string
        sdf_string = cp.stdout

        # There might be warnings in the output, so we remove them by finding the
        # first <sdf> tag and ignoring everything before it
        sdf_string = sdf_string[sdf_string.find("<sdf") :]

        return sdf_string
