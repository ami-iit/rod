import abc
import dataclasses
import numbers
from typing import Any, List, Tuple


class DataclassPrettyPrinter(abc.ABC):
    def to_string(self) -> str:
        return DataclassPrettyPrinter.dataclass_to_str(obj=self, level=1)

    @staticmethod
    def list_to_string(obj: List[Any], level: int = 1) -> str:
        if not isinstance(obj, list):
            raise TypeError(obj, type(obj))

        if all(isinstance(el, (numbers.Number, str)) for el in obj):
            return str(obj)

        spacing = " " * 4
        spacing_level = spacing * level
        spacing_level_up = spacing * (level - 1)

        list_str = [
            DataclassPrettyPrinter.dataclass_to_str(obj=el, level=level + 1)
            if dataclasses.is_dataclass(el)
            else str(el)
            for el in obj
        ]

        return (
            f"[\n"
            + ",\n".join(f"{spacing_level}{el!s}" for el in list_str)
            + f",\n{spacing_level_up}]"
        )

    @staticmethod
    def dataclass_to_str(obj: Any, level: int = 1) -> str:
        if not dataclasses.is_dataclass(obj):
            raise TypeError(obj, type(obj))

        serialization: List[Tuple[str, str]] = []

        for field in dataclasses.fields(obj):
            attr = getattr(obj, field.name)

            if attr is None or attr == "":
                continue

            elif isinstance(attr, list):
                list_str = DataclassPrettyPrinter.list_to_string(
                    obj=attr, level=level + 1
                )
                serialization += [(field.name, list_str)]
                continue

            elif dataclasses.is_dataclass(attr):
                dataclass_str = DataclassPrettyPrinter.dataclass_to_str(
                    obj=attr, level=level + 1
                )
                serialization += [(field.name, dataclass_str)]
                continue

            else:
                serialization += [(field.name, f"{attr!s}")]
                continue

        spacing = " " * 4
        spacing_level = spacing * level
        spacing_level_up = spacing * (level - 1)

        self_str = ",\n".join(
            f"{spacing_level}{name}={value_str}" for name, value_str in serialization
        )
        return f"{type(obj).__name__}(\n{self_str},\n{spacing_level_up})"
