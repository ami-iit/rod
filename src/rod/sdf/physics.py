import dataclasses
from typing import Optional

import mashumaro

from .element import Element


@dataclasses.dataclass
class Physics(Element):
    name: Optional[str] = dataclasses.field(
        default=None, metadata=mashumaro.field_options(alias="@name")
    )

    default: Optional[bool] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(
            alias="@default",
            serialize=Element.serialize_bool,
            deserialize=Element.deserialize_bool,
        ),
    )

    type: str = dataclasses.field(
        default="ode", metadata=mashumaro.field_options(alias="@type")
    )

    max_step_size: float = dataclasses.field(
        default=0.001,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    real_time_factor: float = dataclasses.field(
        default=1.0, metadata=mashumaro.field_options(serialize=Element.serialize_float)
    )

    real_time_update_rate: float = dataclasses.field(
        default=1000.0,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )

    max_contacts: Optional[float] = dataclasses.field(
        default=None,
        metadata=mashumaro.field_options(serialize=Element.serialize_float),
    )
