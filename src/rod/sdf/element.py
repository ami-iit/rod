import dataclasses
from typing import Any, Dict, List, Optional

import mashumaro.config
import mashumaro.mixins.dict
import numpy as np

from rod.pretty_printer import DataclassPrettyPrinter


@dataclasses.dataclass
class Element(mashumaro.mixins.dict.DataClassDictMixin, DataclassPrettyPrinter):
    class Config(mashumaro.config.BaseConfig):
        serialize_by_alias = True

    def __post_serialize__(self, d: Dict[Any, Any]) -> Dict[Any, Any]:
        out = d.copy()

        for key, value in d.items():
            if value is None or value == "":
                _ = out.pop(key)

        return out

    def __str__(self) -> str:
        return self.to_string()

    @staticmethod
    def serialize_bool(data: bool) -> str:
        assert isinstance(data, bool)
        return "false" if data is False else "true"

    @staticmethod
    def deserialize_bool(data: str) -> bool:
        assert isinstance(data, str)
        true_vals = {"1", "True", "true"}
        false_vals = {"0", "False", "false"}
        assert data in true_vals.union(false_vals)

        return True if data in true_vals else False

    @staticmethod
    def serialize_float(data: float) -> str:
        assert isinstance(data, float)
        return str(data)

    @staticmethod
    def serialize_list(data: List[float]) -> str:
        assert isinstance(data, list)
        return " ".join(np.array(data, dtype=str))

    @staticmethod
    def deserialize_list(data: str, length: Optional[int] = None) -> List[float]:
        assert isinstance(data, str)
        array = np.atleast_1d(np.array(data.split(sep=" "), dtype=float).squeeze())

        if length is not None:
            assert array.size == length

        return list(array)
