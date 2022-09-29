import dataclasses
import pathlib
from typing import List, Optional, Union

import mashumaro
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
    def load(sdf: Union[pathlib.Path, str]) -> "Sdf":
        """
        Load an SDF resource.

        The SDF resource could either be a `pathlibPath` or a `str` containing
        either the path to the SDF file or an already parsed SDF string.

        Args:
            sdf: The SDF resource to load.

        Returns:
            The parsed SDF file.
        """

        try:
            from ctypes.wintypes import MAX_PATH
        except ValueError:
            import os

            MAX_PATH = os.pathconf("/", "PC_PATH_MAX")

        if isinstance(sdf, pathlib.Path):
            sdf_string = sdf.read_text()

        elif (
            isinstance(sdf, str)
            and len(sdf) <= MAX_PATH
            and pathlib.Path(sdf).is_file()
        ):
            sdf_string = pathlib.Path(sdf).read_text()

        else:
            sdf_string = sdf

        try:
            xml_dict = xmltodict.parse(xml_input=sdf_string)
        except Exception:
            raise ValueError("Failed to parse 'sdf' argument")

        try:
            sdf_dict = xml_dict["sdf"]
        except KeyError:
            raise RuntimeError("Failed to find top-level '<sdf>' element")

        return Sdf.from_dict(sdf_dict)

    def serialize(self, pretty: bool = False, indent: str = "  "):

        return xmltodict.unparse(
            input_dict=dict(sdf=self.to_dict()), pretty=pretty, indent=indent
        )
