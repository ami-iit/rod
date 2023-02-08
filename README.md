# RObot Description processor

**The ultimate Python tool for RObot Descriptions processing.**

ROD is yet another library to operate on robot descriptions based on the [SDFormat][sdformat] specification.

## Why SDFormat?

Among the many existing robot description formats, SDFormat provides a well-defined and maintained [versioned specification][sdformat_spec] that controls the available fields and their content.
[Open Robotics][open_robotics] already provides the C++ library [`gazebosim/sdformat`](https://github.com/gazebosim/sdformat) with initial support of [Python bindings][sdformat_python].
However, C++ dependencies in pure-Python projects are typically quite complicated to handle and maintain.
Here ROD comes to rescue.

URDF, thanks to native ROS support, is historically the most popular robot description used by the community.
The main problem of URDF is that it is not a specification, and developers of URDF descriptions might produce models and parsers that do not comply to any standard.
Luckily, URDF models can be easily converted to SDF[^urdf_to_sdf].
If the URDF model is not compliant, the process errors with clear messages.
Furthermore, modern versions of the converter produce a SDF description with standardized [pose semantics][pose_semantics],
that greatly simplifies the life of downstream developers that do not have to guess the reference frame or pose elements.
Last but not least, the pose semantics also makes SDF aware of the concept of _frame_ that URDF is missing.

## Features

- Out-of-the-box support of SDFormat specifications [≥ 1.7][sdformat_spec_17]
- Serialization and deserialization support of SDF files
- In-memory layout based on `dataclasses`
- Syntax highlighting and auto-completion
- Support of programmatic creation of SDF files from Python APIs
- Transitive support of URDF through conversion to SDF[^urdf_to_sdf]
- Type validation of elements and attributes
- Automatic check of missing required elements
- Based on [`Fatal1ty/mashumaro`][mashumaro] for great serialization and deserialization performance
- Support of exporting the in-memory model description to URDF

[mashumaro]: https://github.com/Fatal1ty/mashumaro
[open_robotics]: https://www.openrobotics.org/
[pose_semantics]: http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&
[sdformat]: http://sdformat.org/
[sdformat_python]: http://sdformat.org/tutorials?tut=python_bindings&cat=developers&
[sdformat_repo]: https://github.com/gazebosim/sdformat
[sdformat_spec]: http://sdformat.org/spec
[sdformat_spec_17]: http://sdformat.org/spec?elem=sdf&ver=1.7
[urdf]: http://wiki.ros.org/urdf

[^urdf_to_sdf]: Conversion can be done either using `ign sdf` included in Ignition Gazebo Fortress, or `gz sdf` included in Gazebo Sim starting from Garden.

## Installation

You can install the project with [`pypa/pip`][pip], preferably in a [virtual environment][venv]:

```bash
pip install git+https://github.com/ami-iit/rod
```

[pip]: https://github.com/pypa/pip/
[venv]: https://docs.python.org/3.8/tutorial/venv.html

## Examples

<details>
<summary>Serialize and deserialize SDF files</summary>

```python
import pathlib

from rod import Sdf

# Supported SDF resources
sdf_resource_1 = "/path/to/file.sdf"
sdf_resource_2 = pathlib.Path(sdf_resource_1)
sdf_resource_3 = sdf_resource_2.read_text()

# Deserialize SDF resources
sdf_1 = Sdf.load(sdf=sdf_resource_1)
sdf_2 = Sdf.load(sdf=sdf_resource_2)
sdf_3 = Sdf.load(sdf=sdf_resource_3)

# Serialize in-memory Sdf object
print(sdf_3.serialize(pretty=True))
```

</details>

<details>
<summary>Create SDF models programmatically</summary>

```python
from rod import Axis, Inertia, Inertial, Joint, Limit, Link, Model, Sdf, Xyz

sdf = Sdf(
    version="1.7",
    model=Model(
        name="my_model",
        link=[
            Link(name="base_link", inertial=Inertial(mass=1.0, inertia=Inertia())),
            Link(name="my_link", inertial=Inertial(mass=0.5, inertia=Inertia())),
        ],
        joint=Joint(
            name="base_to_my_link",
            type="revolute",
            parent="base_link",
            child="my_link",
            axis=Axis(xyz=Xyz(xyz=[0, 0, 1]), limit=Limit(lower=-3.13, upper=3.14)),
        ),
    ),
)

print(sdf.serialize(pretty=True))
```

```xml
<?xml version="1.0" encoding="utf-8"?>
<sdf version="1.7">
  <model name="my_model">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <iyy>1.0</iyy>
          <izz>1.0</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
    </link>
    <link name="my_link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>1.0</ixx>
          <iyy>1.0</iyy>
          <izz>1.0</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
    </link>
    <joint name="base_to_my_link" type="revolute">
      <parent>base_link</parent>
      <child>my_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-3.13</lower>
          <upper>3.14</upper>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

</details>

<details>
<summary>Exporting SDF to URDF</summary>

```python
# Generate first the 'sdf' object with the collapsed code
# of the section 'Create SDF models programmatically'.

from rod.urdf.exporter import UrdfExporter

urdf_string = UrdfExporter.sdf_to_urdf_string(
    sdf=sdf,
    pretty=True,
    gazebo_preserve_fixed_joints=True,
)

print(urdf_string)
```

```xml
<?xml version="1.0" encoding="utf-8"?>
<robot name="my_model">
  <link name="base_link">
    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="my_link">
    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <mass value="0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="base_to_my_link" type="revolute">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="base_link"/>
    <child link="my_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="3.4028235e+38" velocity="3.4028235e+38" lower="-3.13" upper="3.14"/>
  </joint>
</robot>
```

</details>

## Similar projects

- https://github.com/gazebosim/sdformat
- https://github.com/mmatl/urdfpy
- https://github.com/clemense/yourdfpy
- https://github.com/ros/urdf_parser_py
- https://github.com/FirefoxMetzger/python-sdformat/

## Contributing

Pull requests are welcome. 
For major changes, please open an issue first to discuss what you would like to change.

## Maintainers

| [<img src="https://github.com/diegoferigo.png" width="40">][df] | [@diegoferigo][df] |
|:---------------------------------------------------------------:|:------------------:|

[df]: https://github.com/diegoferigo

## License

[BSD3](https://choosealicense.com/licenses/bsd-3-clause/)
