from __future__ import annotations

import dataclasses
import pathlib

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

    world: World | list[World] | None = dataclasses.field(default=None)

    model: Model | list[Model] | None = dataclasses.field(default=None)

    def worlds(self) -> list[World]:
        if self.world is None:
            return []

        if isinstance(self.world, World):
            return [self.world]

        assert isinstance(self.world, list), type(self.world)
        return self.world

    def models(self) -> list[Model]:
        if self.model is None:
            return []

        if isinstance(self.model, Model):
            return [self.model]

        assert isinstance(self.model, list), type(self.model)
        return self.model

    @staticmethod
    def load(sdf: pathlib.Path | str, is_urdf: bool | None = None) -> Sdf:
        """
        Load an SDF resource.

        The SDF resource could either be a `pathlibPath` or a `str` containing
        either the path to the SDF file or an already parsed SDF string.

        Args:
            sdf: The SDF resource to load.
            is_urdf: Force the SDF resource to be treated as URDF if the automatic detection fails.

        Returns:
            The parsed SDF file.
        """

        path = pathlib.Path(sdf)

        match sdf:
            # Case 1: Handle strings.
            case str():
                if path.suffix:
                    # Assuming that if the string has a suffix, it is a path.
                    sdf_string = pathlib.Path(sdf).read_text(encoding="utf-8")
                else:
                    # Otherwise, it is an SDF string.
                    sdf_string = sdf

                is_urdf = is_urdf if is_urdf is not None else "<robot" in sdf_string

            # Case 2: Handle pathlib.Path.
            case pathlib.Path():
                sdf_string = path.read_text(encoding="utf-8")
                is_urdf = is_urdf if is_urdf is not None else path.suffix == ".urdf"

            # Case 3: Raise an error for unsupported types.
            case _:
                raise TypeError(f"Unsupported type for 'sdf': {type(sdf)}")

        # Convert SDF to URDF if needed (it requires system executables)
        if is_urdf:
            urdf_string = sdf_string
            sdf_string = GazeboHelper.process_model_description_with_sdformat(
                model_description=urdf_string
            )

        # Parse the SDF to dict
        try:
            xml_dict = xmltodict.parse(xml_input=sdf_string)
        except Exception as exc:
            raise RuntimeError("Failed to parse 'sdf' argument") from exc

        # Look for the top-level <sdf> element
        try:
            sdf_dict = xml_dict["sdf"]
        except KeyError as exc:
            raise RuntimeError("Failed to find top-level '<sdf>' element") from exc

        # Get the SDF version
        sdf = Sdf.from_dict(sdf_dict)
        sdf_version = packaging.version.Version(sdf_dict["@version"])

        # Check that the SDF version is compatible
        if sdf_version not in packaging.specifiers.SpecifierSet(">= 1.7"):
            raise RuntimeError(f"Unsupported SDF version: {sdf_version}")

        return sdf

    def serialize(
        self, pretty: bool = False, indent: str = "  ", validate: bool | None = None
    ) -> str:
        # Automatically detect suitable Gazebo version
        validate = validate if validate is not None else GazeboHelper.has_gazebo()

        if validate:
            _ = GazeboHelper.process_model_description_with_sdformat(
                model_description=xmltodict.unparse(
                    input_dict={"sdf": self.to_dict()}, pretty=True, indent="  "
                )
            )

        return xmltodict.unparse(
            input_dict={"sdf": self.to_dict()},
            pretty=pretty,
            indent=indent,
            short_empty_elements=True,
        )
