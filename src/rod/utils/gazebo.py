import functools
import hashlib
import os
import pathlib
import shutil
import subprocess
import tempfile
from typing import ClassVar


class GazeboHelper:
    _cached_executable: pathlib.Path | None = None
    _process_cache: ClassVar[dict[str, str]] = {}
    _max_cache_size: int = 100

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
        except Exception:
            return False

    @staticmethod
    @functools.lru_cache(maxsize=128)
    def _get_cached_processing(content_hash: str, content: str) -> str:
        """Internal cached processing method."""
        return GazeboHelper._process_with_sdformat_uncached(content)

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
        # Use caching for repeated content
        # ================================

        # Create hash of content for caching
        content_hash = hashlib.md5(model_description_string.encode()).hexdigest()

        # Try to get from cache first
        return GazeboHelper._get_cached_processing(
            content_hash, model_description_string
        )

    @staticmethod
    def _process_with_sdformat_uncached(model_description_string: str) -> str:
        """Internal method for actual SDF processing without caching."""
        # Get the Gazebo Sim executable (raises exception if not found)
        gazebo_executable = GazeboHelper.get_gazebo_executable()

        # Use faster temporary file approach - write directly to /tmp on Unix systems
        # This avoids Python's overhead of TemporaryDirectory context manager
        # As soon as 3.12 will be the minimum supported version, we can use just
        # NamedTemporaryFile with the new delete_on_close=False parameter.
        if os.name == "posix":  # Unix-like systems
            tmp_dir = pathlib.Path("/tmp")
        else:
            tmp_dir = pathlib.Path(tempfile.gettempdir())

        temp_file = (
            tmp_dir
            / f"rod_sdf_{os.getpid()}_{hash(model_description_string) & 0x7fffffff}.xml"
        )

        try:
            # Write file directly
            temp_file.write_text(model_description_string, encoding="utf-8")

            # Process with optimized subprocess call
            try:
                cp = subprocess.run(
                    [str(gazebo_executable), "sdf", "-p", str(temp_file)],
                    text=True,
                    capture_output=True,
                    check=True,
                    # Add performance optimizations
                    bufsize=-1,  # Use system default buffer size
                    env=dict(
                        os.environ,
                        **{
                            "IGN_PARTITION": "rod_processing",  # Avoid interference with running Gazebo
                            "GZ_PARTITION": "rod_processing",
                        },
                    ),
                )
            except subprocess.CalledProcessError as e:
                if e.returncode != 0:
                    print(e.stdout)
                    raise RuntimeError(
                        "Failed to process the input with sdformat"
                    ) from e

        finally:
            # Clean up temp file
            if temp_file.exists():
                temp_file.unlink()

        # Get the resulting SDF string
        sdf_string = cp.stdout

        # There might be warnings in the output, so we remove them by finding the
        # first <sdf> tag and ignoring everything before it
        sdf_start = sdf_string.find("<sdf")
        if sdf_start != -1:
            sdf_string = sdf_string[sdf_start:]

        return sdf_string

    @staticmethod
    def process_multiple_descriptions(
        descriptions: list[str | pathlib.Path],
    ) -> list[str]:
        """Process multiple SDF descriptions in batch for better performance."""
        results = []
        for desc in descriptions:
            results.append(GazeboHelper.process_model_description_with_sdformat(desc))
        return results

    @classmethod
    def clear_cache(cls) -> None:
        """Clear the processing cache to free memory."""
        cls._get_cached_processing.cache_clear()
        cls._process_cache.clear()
