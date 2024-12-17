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

- Out-of-the-box support for SDFormat specifications [≥ 1.10][sdformat_spec_110].
- Serialization and deserialization support for SDF files.
- In-memory layout based on `dataclasses`.
- Syntax highlighting and auto-completion.
- Programmatic creation of SDF files from Python APIs.
- Transitive support for URDF through conversion to SDF.
- Type validation of elements and attributes.
- Automatic check of missing required elements.
- High-performance serialization and deserialization using [`Fatal1ty/mashumaro`][mashumaro].
- Export in-memory model description to URDF.

[mashumaro]: https://github.com/Fatal1ty/mashumaro
[open_robotics]: https://www.openrobotics.org/
[pose_semantics]: http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&
[sdformat]: http://sdformat.org/
[sdformat_python]: http://sdformat.org/tutorials?tut=python_bindings&cat=developers&
[sdformat_spec]: http://sdformat.org/spec
[sdformat_spec_110]: http://sdformat.org/spec?elem=sdf&ver=1.10
[urdf]: http://wiki.ros.org/urdf

[^urdf_to_sdf]: Conversion can be done using the `gz sdf` command included in Gazebo Sim starting from Garden.

## Installation

> [!TIP]
> ROD does not support out-of-the-box URDF files.
> URDF support is obtained by converting URDF files to SDF using the `gz sdf` command provided by [sdformat][sdformat_repo] and [gz-tools][gz-tools_repo].
> Ensure these tools are installed on your system if URDF support is needed (more information below).

[sdformat_repo]: https://github.com/gazebosim/sdformat
[gz-tools_repo]: https://github.com/gazebosim/gz-tools

<details>
<summary>Using conda (recommended)</summary>

Installing ROD using `conda` is the recommended way to obtain a complete installation with out-of-the-box support for both URDF and SDF descriptions:

```bash
conda install rod -c conda-forge
```

This will automatically install `sdformat` and `gz-tools`.

</details>

<details>
<summary>Using pixi</summary>

[`pixi`](https://pixi.sh) definetly provides the quickest way to start using ROD. You can run the tests by executing:

```bash
pixi run test
```

or install the default dependencies with:

```bash
pixi install
```

check out the [pyproject.toml](./pyproject.toml) file for the list of all available features and read the [`pixi`](https://pixi.sh) documentation for more information.

</details>


<details>
<summary>Using pip</summary>

You can install ROD from PyPI with [`pypa/pip`][pip], preferably in a [virtual environment][venv]:

```bash
pip install rod[all]
```

If you need URDF support, follow the [official instructions][gazebo_sim_docs] to install Gazebo Sim on your operating system,
making sure to obtain `sdformat ≥ 13.0` and `gz-tools ≥ 2.0`.

You don't need to install the entire Gazebo Sim suite.
For example, on Ubuntu, you can only install the `libsdformat13 gz-tools2` packages.

[pip]: https://github.com/pypa/pip/
[venv]: https://docs.python.org/3.10/tutorial/venv.html
[gazebo_sim_docs]: https://gazebosim.org/docs

</details>

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

urdf_string = UrdfExporter(pretty=True, gazebo_preserve_fixed_joints=True).to_urdf_string(
    sdf=sdf
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

## Command Line Interface

ROD provides a command line interface (CLI) for processing robot description files. You can use the CLI to display file attributes or convert files between URDF and SDF formats.

### Usage

- Display the attributes of an SDF file:

```sh
rod --file /path/to/file.sdf --show
```

- Convert an SDF file to URDF:

```sh
rod --file /path/to/file.sdf -o /path/to/file.urdf
```

## Similar projects

- https://github.com/gazebosim/sdformat
- https://github.com/mmatl/urdfpy
- https://github.com/clemense/yourdfpy
- https://github.com/ros/urdf_parser_py
- https://github.com/FirefoxMetzger/python-sdformat/

## Contributing

Pull requests are welcome.
For major changes, please open an issue first to discuss what you would like to change.

## People

### Author

| [<img src="https://github.com/diegoferigo.png" width="40">][df] | [@diegoferigo][df] |
|:---------------------------------------------------------------:|:------------------:|

[df]: https://github.com/diegoferigo

### Maintainers

| [<img src="https://github.com/flferretti.png" width="40">][ff] | [@flferretti][ff] | [<img src="https://github.com/CarlottaSartore.png" width="40">][cs] | [@CarlottaSartore][cs] |
|:---------------------------------------------------------------:|:------------------:|:---------------------------------------------------------------:|:------------------:|



[ff]: https://github.com/flferretti
[cs]: https://github.com/CarlottaSartore

## License

[BSD3](https://choosealicense.com/licenses/bsd-3-clause/)
