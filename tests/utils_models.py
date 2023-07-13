import enum
import importlib
import pathlib
from typing import Union


class Robot(enum.IntEnum):
    Hyq = enum.auto()
    iCub = enum.auto()
    Ur10 = enum.auto()
    Baxter = enum.auto()
    Cassie = enum.auto()
    Fetch = enum.auto()
    AtlasV4 = enum.auto()
    Laikago = enum.auto()
    AnymalC = enum.auto()
    Ergocub = enum.auto()
    Talos = enum.auto()
    Valkyrie = enum.auto()
    DoublePendulum = enum.auto()
    SimpleHumanoid = enum.auto()

    def to_module_name(self) -> str:
        """"""

        if self is Robot.iCub:
            return "icub_description"

        # Convert the camelcase robot name to snakecase
        name_sc = "".join(
            ["_" + c.lower() if c.isupper() else c for c in self.name]
        ).lstrip("_")

        return f"{name_sc}_description"


class ModelFactory:
    """Factory class providing URDF files used by the tests."""

    @staticmethod
    def get_model_description(robot: Union[Robot, str]) -> pathlib.Path:
        """
        Get the URDF file of different robots.

        Args:
            robot: Robot name of the desired URDF file.

        Returns:
            Path to the URDF file of the robot.
        """

        module_name = robot if isinstance(robot, str) else robot.to_module_name()
        module = importlib.import_module(f"robot_descriptions.{module_name}")

        urdf_path = pathlib.Path(module.URDF_PATH)

        if not urdf_path.is_file():
            raise FileExistsError(f"URDF file '{urdf_path}' does not exist")

        return urdf_path
