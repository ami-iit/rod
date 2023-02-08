import dataclasses
import os
import pathlib
from typing import List, Optional, Union

import mashumaro
import packaging.specifiers
import packaging.version
import xmltodict

from rod.utils.gazebo import GazeboHelper

from .element import Element
from .model import Model
from .world import World


@dataclasses.dataclass
class Sdf(Element):
    version: str = dataclasses.field(metadata=mashumaro.field_options(alias="@version"))

    world: Optional[Union[World, List[World]]] = dataclasses.field(default=None)

    model: Optional[Union[Model, List[Model]]] = dataclasses.field(default=None)

    def worlds(self) -> List[World]:
        if self.world is None:
            return []

        if isinstance(self.world, World):
            return [self.world]

        assert isinstance(self.world, list), type(self.world)
        return self.world

    def models(self) -> List[Model]:
        if self.model is None:
            return []

        if isinstance(self.model, Model):
            return [self.model]

        assert isinstance(self.model, list), type(self.model)
        return self.model

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
            sdf_string = GazeboHelper.process_model_description_with_sdformat(
                model_description=urdf_string
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

    def serialize(
        self, pretty: bool = False, indent: str = "  ", validate: Optional[bool] = None
    ) -> str:
        # Automatically detect suitable Gazebo version
        validate = validate if validate is not None else GazeboHelper.has_gazebo()

        if validate:
            _ = GazeboHelper.process_model_description_with_sdformat(
                model_description=xmltodict.unparse(
                    input_dict=dict(sdf=self.to_dict()), pretty=True, indent="  "
                )
            )

        return xmltodict.unparse(
            input_dict=dict(sdf=self.to_dict()),
            pretty=pretty,
            indent=indent,
            short_empty_elements=True,
        )
