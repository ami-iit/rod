import os
import pathlib
import shutil
import subprocess
import tempfile


class GazeboHelper:
    _cached_executable: pathlib.Path = None

    @classmethod
    def get_gazebo_executable(cls) -> pathlib.Path:
        if cls._cached_executable is not None:
            return cls._cached_executable

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
        try:
            subprocess.run(
                [executable, "sdf", "--help"], check=True, capture_output=True
            )
        except subprocess.CalledProcessError as e:
            msg = f"Failed to find 'sdf' command part of {executable} installation"
            raise RuntimeError(msg) from e

        cls._cached_executable = executable

        return cls._cached_executable

    @staticmethod
    def has_gazebo() -> bool:
        try:
            _ = GazeboHelper.get_gazebo_executable()
            return True
        except:
            return False

    @staticmethod
    def process_model_description_with_sdformat(
        model_description: str | pathlib.Path,
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
            model_description_string = pathlib.Path(model_description).read_text(
                encoding="utf-8"
            )

        # Finally, it must be a SDF/URDF string
        else:
            model_description_string = model_description

        # ================================
        # Process the string with sdformat
        # ================================

        # Get the Gazebo Sim executable (raises exception if not found)
        gazebo_executable = GazeboHelper.get_gazebo_executable()

        # Operate on a file stored in a temporary directory.
        # This is necessary on windows because the file has to be closed before
        # it can be processed by the sdformat executable.
        # As soon as 3.12 will be the minimum supported version, we can use just
        # NamedTemporaryFile with the new delete_on_close=False parameter.
        with tempfile.TemporaryDirectory() as tmp:

            with tempfile.NamedTemporaryFile(
                mode="w+", suffix=".xml", dir=tmp, delete=False
            ) as fp:

                fp.write(model_description_string)
                fp.close()

            try:
                cp = subprocess.run(
                    [str(gazebo_executable), "sdf", "-p", fp.name],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            except subprocess.CalledProcessError as e:
                if cp.returncode != 0:
                    print(cp.stdout)
                    raise RuntimeError(
                        "Failed to process the input with sdformat"
                    ) from e

        # Get the resulting SDF string
        sdf_string = cp.stdout

        # There might be warnings in the output, so we remove them by finding the
        # first <sdf> tag and ignoring everything before it
        sdf_string = sdf_string[sdf_string.find("<sdf") :]

        return sdf_string
