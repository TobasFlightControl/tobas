# What Is UADF?

The Universal Aircraft Description Format (UADF), defined by Tobas,
extends the Universal Robot Description Format (URDF), a common format for representing robots,
with aircraft-specific elements.

## What Is URDF?

---

URDF is a format for describing any robot that can be represented as a system of rigid links.
It defines the mass properties, collision geometry, and visual geometry of each link, as well as the joints connecting them.
For details, see <a href=https://wiki.ros.org/urdf target="_blank">urdf | ROS.org</a>.

## Differences Between UADF and URDF

---

In addition to the joint types in URDF, UADF provides the following aircraft-specific joint types.

### thrust

A joint representing a propulsion unit consisting of a motor and a propeller.
It is based on URDF's `continuous`, with the following differences.

- `axis`: The rotation axis. Thrust is assumed to act along this axis.
- `direction`: The rotation direction. Set `value` to `cw` or `ccw`.

!!! note

    A `thrust` joint must be a terminal joint.

### cs

A joint representing a control surface on a fixed-wing aircraft.
It is based on URDF's `revolute`.

!!! note

    A `cs` joint must be a terminal joint.

### tilt

Represents the tilt joint of an active tilt rotor.
It is based on URDF's `revolute`.

!!! note

    A `tilt` joint must have exactly one `thrust` joint connected downstream.

## How to Create a UADF

---

Here, we will create a UADF for the DJI F450, the typical quadcopter used in the tutorial.

![f450](../../assets/what_is_uadf/f450.png)

URDF supports mesh and texture files for visualization,
and these files are often distributed along with the URDF itself,
so it is useful to create a dedicated ROS package for each robot.
For this example, we will create a package named ` tobas_f450_description` directly under `~/colcon_ws/src/` and include all the necessary files in it.
The completed package is available at
<a href=https://github.com/TobasFlightControl/tobas_f450_description target="_blank">TobasFlightControl/tobas_f450_description</a>.

### ROS Package Structure and Required Files

`tobas_f450_description` has the following structure.

```text
tobas_f450_description/
├── meshes
│   ├── frame.stl
│   ├── phantom3_0945_ccw.stl
│   ├── phantom3_0945_cw.stl
│   └── tobas_fc100.stl
├── urdf
│   └── f450.uadf
├── CMakeLists.txt
└── package.xml
```

`package.xml` describes the package and its dependencies.
Each ROS package must have one at its root.
Since this package has no external package dependencies, it contains only essential elements such as the package name and license.

```xml
<package format="3">
	<name>tobas_f450_description</name>
	<version>0.0.0</version>
	<description>A UADF example for the DJI F450.</description>
	<maintainer email="m.dohi@tobas.jp">Masayoshi Dohi</maintainer>
	<license>MIT</license>
	<buildtool_depend>ament_cmake</buildtool_depend>
	<export>
		<build_type>ament_cmake</build_type>
	</export>
</package>
```

`CMakeLists.txt` specifies the package's build and installation procedures.
Configure it to install `meshes`, the directory containing the mesh files, and `urdf`, the directory containing the UADF.

```cmake
cmake_minimum_required(VERSION 3.25)
project(tobas_f450_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})
ament_package()
```

### Writing a UADF

The F450 UADF we will create consists of the following components.

- An empty root link (required)
- The main frame, including the motors
- A battery fixed to the main frame
- An FMU fixed to the main frame
- Four propellers

Each component is described as a single rigid link,
and the links are connected by appropriate joints to form a single tree structure.

In this example, the main frame and all links below it will be attached under the root link.
The overall tree structure is as follows.

```text
base_link
├──frame
├──battery
├──fmu
├──propeller_0
├──propeller_1
├──propeller_2
└──propeller_3
```

First, define the URDF root element.
The URDF root element must be `robot` and must have a `name` attribute.
All the following elements go inside this root element.

```xml
<robot name="f450">
</robot>
```

Define an empty root link inside `robot`.
The root link is a conceptual link with no size or mass, and every UADF must have exactly one root link.

```xml
  <!-- Base Link (Empty) -->
  <link name="base_link"/>
```

Fix the main frame to the root link.
The mesh file in `visual` is specified using a path relative to the package.
You can use `package://<package name>` to specify paths relative to a ROS package in this way.
The collision geometry in `collision` is approximated by a box, since using a mesh file would slow down the simulation.
The mass properties in `inertial` are copied directly from 3D CAD data.

```xml
  <!-- Main Frame -->
  <joint name="frame_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="frame"/>
  </joint>
  <link name="frame">
    <inertial>
      <mass value="0.55346"/>
      <origin xyz="0 0 -0.004136" rpy="0 0 0"/>
      <inertia ixx="0.008128" ixy="0" ixz="0" iyy="0.008193" iyz="0" izz="0.01597"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/frame.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.03" rpy="0 0 0"/>
      <geometry>
        <box size="0.35 0.35 0.06"/>
      </geometry>
    </collision>
  </link>
```

Add the battery and FMU in the same way as the main frame.
For each component, calculate and enter the moments of inertia assuming that its mass is uniformly distributed in a box with the same dimensions as `collision`.
For details on the calculations, see [Inertia Moment](./inertia_moment.md).

```xml
  <!-- Battery -->
  <joint name="battery_joint" type="fixed">
    <origin xyz="0 0 -0.025" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="battery"/>
  </joint>
  <link name="battery">
    <inertial>
      <mass value="0.376"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="9.024e-05" ixy="0" ixz="0" iyy="0.000770831" iyz="0" izz="0.000824975"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
    </collision>
  </link>

  <!-- Flight Management Unit -->
  <joint name="fmu_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="fmu"/>
  </joint>
  <link name="fmu">
    <inertial>
      <mass value="0.16"/>
      <origin xyz="0 0 0.014" rpy="0 0 0"/>
      <inertia ixx="6.17067e-05" ixy="0" ixz="0" iyy="0.000118453" iyz="0" izz="0.000159253"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/tobas_fc100.stl" scale="1 1 1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.014" rpy="0 0 0"/>
      <geometry>
        <box size="0.09 0.062 0.028"/>
      </geometry>
    </collision>
  </link>
```

Add a propeller.
Set the joint type to the UADF-specific `thrust`, set `axis` to the +Z axis direction (`0 0 1`), which is the thrust direction, and set `direction` to counterclockwise (`ccw`).
The mesh file in `visual` is specified using a path relative to the package.
The collision geometry in `collision` is approximated by a cylinder.
The moments of inertia in `inertial` are calculated assuming that the mass is uniformly distributed in a cylinder with the same dimensions as `collision`.
For details on the calculations, see [Inertia Moment](./inertia_moment.md).

```xml
  <!-- Propeller (0) -->
  <joint name="propeller_0_joint" type="thrust">
    <origin xyz="0.159099 -0.159099 0.0247" rpy="0 0 -0.785398"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="propeller_0"/>
    <direction value="ccw"/>
  </joint>
  <link name="propeller_0">
    <inertial>
      <mass value="0.011"/>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <inertia ixx="3.9732e-05" ixy="0" ixz="0" iyy="3.9732e-05" iyz="0" izz="7.92e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/phantom3_0945_ccw.stl" scale="1 1 1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.12" length="0.012"/>
      </geometry>
    </collision>
  </link>
```

Add the other three propellers in the same way.
Note that each has a different joint origin, rotation direction, and mesh path.

The final UADF is shown below.

```xml
<robot name="f450">

  <!-- Base Link (Empty) -->
  <link name="base_link"/>

  <!-- Main Frame -->
  <joint name="frame_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="frame"/>
  </joint>
  <link name="frame">
    <inertial>
      <mass value="0.55346"/>
      <origin xyz="0 0 -0.004136" rpy="0 0 0"/>
      <inertia ixx="0.008128" ixy="0" ixz="0" iyy="0.008193" iyz="0" izz="0.01597"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/frame.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.03" rpy="0 0 0"/>
      <geometry>
        <box size="0.35 0.35 0.06"/>
      </geometry>
    </collision>
  </link>

  <!-- Battery -->
  <joint name="battery_joint" type="fixed">
    <origin xyz="0 0 -0.025" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="battery"/>
  </joint>
  <link name="battery">
    <inertial>
      <mass value="0.376"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="9.024e-05" ixy="0" ixz="0" iyy="0.000770831" iyz="0" izz="0.000824975"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
    </collision>
  </link>

  <!-- Flight Management Unit -->
  <joint name="fmu_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="fmu"/>
  </joint>
  <link name="fmu">
    <inertial>
      <mass value="0.16"/>
      <origin xyz="0 0 0.014" rpy="0 0 0"/>
      <inertia ixx="6.17067e-05" ixy="0" ixz="0" iyy="0.000118453" iyz="0" izz="0.000159253"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/tobas_fc100.stl" scale="1 1 1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.014" rpy="0 0 0"/>
      <geometry>
        <box size="0.09 0.062 0.028"/>
      </geometry>
    </collision>
  </link>

  <!-- Propeller (0) -->
  <joint name="propeller_0_joint" type="thrust">
    <origin xyz="0.159099 -0.159099 0.0247" rpy="0 0 -0.785398"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="propeller_0"/>
    <direction value="ccw"/>
  </joint>
  <link name="propeller_0">
    <inertial>
      <mass value="0.011"/>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <inertia ixx="3.9732e-05" ixy="0" ixz="0" iyy="3.9732e-05" iyz="0" izz="7.92e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/phantom3_0945_ccw.stl" scale="1 1 1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.12" length="0.012"/>
      </geometry>
    </collision>
  </link>

  <!-- Propeller (1) -->
  <joint name="propeller_1_joint" type="thrust">
    <origin xyz="-0.159099 0.159099 0.0247" rpy="0 0 2.35619"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="propeller_1"/>
    <direction value="ccw"/>
  </joint>
  <link name="propeller_1">
    <inertial>
      <mass value="0.011"/>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <inertia ixx="3.9732e-05" ixy="0" ixz="0" iyy="3.9732e-05" iyz="0" izz="7.92e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/phantom3_0945_ccw.stl" scale="1 1 1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.12" length="0.012"/>
      </geometry>
    </collision>
  </link>

  <!-- Propeller (2) -->
  <joint name="propeller_2_joint" type="thrust">
    <origin xyz="0.159099 0.159099 0.0247" rpy="0 0 0.785398"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="propeller_2"/>
    <direction value="cw"/>
  </joint>
  <link name="propeller_2">
    <inertial>
      <mass value="0.011"/>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <inertia ixx="3.9732e-05" ixy="0" ixz="0" iyy="3.9732e-05" iyz="0" izz="7.92e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/phantom3_0945_cw.stl" scale="1 1 1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.12" length="0.012"/>
      </geometry>
    </collision>
  </link>

  <!-- Propeller (3) -->
  <joint name="propeller_3_joint" type="thrust">
    <origin xyz="-0.159099 -0.159099 0.0247" rpy="0 0 -2.35619"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="propeller_3"/>
    <direction value="cw"/>
  </joint>
  <link name="propeller_3">
    <inertial>
      <mass value="0.011"/>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <inertia ixx="3.9732e-05" ixy="0" ixz="0" iyy="3.9732e-05" iyz="0" izz="7.92e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_f450_description/meshes/phantom3_0945_cw.stl" scale="1 1 1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.006" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.12" length="0.012"/>
      </geometry>
    </collision>
  </link>

</robot>
```

## UADF Examples

---

The example above creates a UADF for a simple quadcopter, but UADF can also represent many other types of aircraft.
Here are a few more UADF examples.

Unlike the previous example, these use XML macros called XACRO to perform the following programming-like operations:

- Include other files
- Define constants
- Perform simple numerical calculations
- Encapsulate repetitive operations in functions

XACRO makes XML shorter and easier to understand.

### Active Tilt Hexacopter (<a href=https://github.com/TobasFlightControl/tobas_voliro_like_description target="_blank">Voliro Like</a>)

This hexacopter has six pairs of `thrust` and `tilt` joints.

![voliro_like](../../assets/what_is_uadf/voliro_like.png)

<details><summary>Before XACRO expansion</summary>

```xml
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="voliro_like">

  <!-- Included URDF Files -->
  <xacro:include filename="$(find tobas_description)/urdf/components/common.xacro"/>
  <xacro:include filename="$(find tobas_description)/urdf/components/fmu/tobas_fc100.xacro"/>
  <xacro:include filename="$(find tobas_description)/urdf/components/battery/hrb_3s_5000mah_50c.xacro"/>
  <xacro:include filename="$(find tobas_description)/urdf/components/landing_gear/zhizicathy_electric_150.xacro"/>
  <xacro:include filename="$(find tobas_voliro_like_description)/urdf/tmotor_1344.uadf"/>

  <!-- Properties -->
  <xacro:property name="arm_offset" value="0.3395"/>

  <!-- Base Link -->
  <link name="base_link"/>

  <!-- Body -->
  <joint name="frame_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="body"/>
  </joint>
  <link name="body">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/frame.stl" scale="1 1 1"/>
      </geometry>
      <xacro:insert_block name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/frame.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.827557"/>
      <origin xyz="1.135e-3 0.003e-3 31.761e-3" rpy="0 0 0"/>
      <inertia ixx="2.486e-3" ixy="1718e-9" ixz="14633e-9" iyy="2.441e-3" iyz="-177e-9" izz="4.396e-3"/>
    </inertial>
  </link>

  <!-- Flight Controller -->
  <xacro:tobas_fc100 parent="base_link">
    <origin xyz="0 0 0.058" rpy="0 0 0"/>
  </xacro:tobas_fc100>

  <!-- Battery -->
  <xacro:hrb_3s_5000mah_50c link_name="battery1" parent="base_link">
    <origin xyz="0 0.06 0.070" rpy="0 0 0"/>
  </xacro:hrb_3s_5000mah_50c>
  <xacro:hrb_3s_5000mah_50c link_name="battery2" parent="base_link">
    <origin xyz="0 -0.06 0.070" rpy="0 0 0"/>
  </xacro:hrb_3s_5000mah_50c>

  <!-- Tilt Rotor -->
  <xacro:macro name="tilt_rotor" params="index direction yaw_deg">
    <joint name="arm_${index}_joint" type="tilt">
      <origin xyz="${arm_offset*cos(yaw_deg*deg2rad)} ${arm_offset*sin(yaw_deg*deg2rad)} 0.0305" rpy="0 0 ${yaw_deg*deg2rad}"/>
      <axis xyz="1 0 0"/>
      <parent link="base_link"/>
      <child link="arm_${index}"/>

      <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
      <limit lower="${-pi}" upper="${pi}" velocity="${106*(2*pi)/60}" effort="1.6"/>
    </joint>

    <link name="arm_${index}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
        </geometry>
        <xacro:insert_block name="black"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="144.803e-3"/>
        <origin xyz="-13.846e-3 -0.014e-3 -1.095e-3" rpy="0 0 0"/>
        <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
      </inertial>
    </link>

    <xacro:tmotor_1344 parent="arm_${index}" index="${index}" direction="${direction}">
      <origin xyz="0 0 0.013" rpy="0 0 0"/>
    </xacro:tmotor_1344>
  </xacro:macro>

  <xacro:tilt_rotor index="0" direction="ccw" yaw_deg="30"/>
  <xacro:tilt_rotor index="1" direction="cw" yaw_deg="90"/>
  <xacro:tilt_rotor index="2" direction="ccw" yaw_deg="150"/>
  <xacro:tilt_rotor index="3" direction="cw" yaw_deg="210"/>
  <xacro:tilt_rotor index="4" direction="ccw" yaw_deg="270"/>
  <xacro:tilt_rotor index="5" direction="cw" yaw_deg="330"/>

  <!-- Landing Gear -->
  <xacro:zhizicathy_electric_150 parent="base_link" distance="0.165" max_angle="${55*deg2rad}">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:zhizicathy_electric_150>

</robot>
```

</details>

<details><summary>After XACRO expansion</summary>

```xml
<robot name="voliro_like">
  <!-- Base Link -->
  <link name="base_link"/>
  <!-- Body -->
  <joint name="frame_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="body"/>
  </joint>
  <link name="body">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/frame.stl" scale="1 1 1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/frame.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.827557"/>
      <origin rpy="0 0 0" xyz="1.135e-3 0.003e-3 31.761e-3"/>
      <inertia ixx="2.486e-3" ixy="1718e-9" ixz="14633e-9" iyy="2.441e-3" iyz="-177e-9" izz="4.396e-3"/>
    </inertial>
  </link>
  <joint name="fmu_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.058"/>
    <parent link="base_link"/>
    <child link="fmu"/>
  </joint>
  <link name="fmu">
    <visual>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/fmu/tobas_fc100.stl" scale="1 1 1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.014"/>
      <geometry>
        <box size="0.09 0.062 0.028"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.16"/>
      <origin rpy="0 0 0" xyz="0 0 0.014"/>
      <inertia ixx="6.170666666666668e-05" ixy="0.0" ixz="0.0" iyy="0.00011845333333333333" iyz="0.0" izz="0.00015925333333333334"/>
    </inertial>
  </link>
  <joint name="battery1_joint" type="fixed">
    <parent link="base_link"/>
    <child link="battery1"/>
    <origin rpy="0 0 0" xyz="0 0.06 0.070"/>
  </joint>
  <link name="battery1">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.376"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.024e-05" ixy="0.0" ixz="0.0" iyy="0.0007708313333333333" iyz="0.0" izz="0.0008249753333333334"/>
    </inertial>
  </link>
  <joint name="battery2_joint" type="fixed">
    <parent link="base_link"/>
    <child link="battery2"/>
    <origin rpy="0 0 0" xyz="0 -0.06 0.070"/>
  </joint>
  <link name="battery2">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.155 0.048 0.024"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.376"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.024e-05" ixy="0.0" ixz="0.0" iyy="0.0007708313333333333" iyz="0.0" izz="0.0008249753333333334"/>
    </inertial>
  </link>
  <joint name="arm_0_joint" type="tilt">
    <origin rpy="0 0 0.5235987755982988" xyz="0.29401562458481695 0.16974999999999998 0.0305"/>
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="arm_0"/>
    <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
    <limit effort="1.6" lower="-3.141592653589793" upper="3.141592653589793" velocity="11.100294042683936"/>
  </joint>
  <link name="arm_0">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="144.803e-3"/>
      <origin rpy="0 0 0" xyz="-13.846e-3 -0.014e-3 -1.095e-3"/>
      <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
    </inertial>
  </link>
  <joint name="propeller_0_joint" type="thrust">
    <parent link="arm_0"/>
    <child link="propeller_0"/>
    <origin rpy="0 0 0" xyz="0 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="ccw"/>
  </joint>
  <link name="propeller_0">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/tmotor_1344_ccw.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.012" radius="0.1651"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0142"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.693633550000001e-05" ixy="0.0" ixz="0.0" iyy="9.693633550000001e-05" iyz="0.0" izz="0.00019353187100000003"/>
    </inertial>
  </link>
  <joint name="arm_1_joint" type="tilt">
    <origin rpy="0 0 1.5707963267948966" xyz="2.078837941552632e-17 0.3395 0.0305"/>
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="arm_1"/>
    <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
    <limit effort="1.6" lower="-3.141592653589793" upper="3.141592653589793" velocity="11.100294042683936"/>
  </joint>
  <link name="arm_1">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="144.803e-3"/>
      <origin rpy="0 0 0" xyz="-13.846e-3 -0.014e-3 -1.095e-3"/>
      <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
    </inertial>
  </link>
  <joint name="propeller_1_joint" type="thrust">
    <parent link="arm_1"/>
    <child link="propeller_1"/>
    <origin rpy="0 0 0" xyz="0 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="cw"/>
  </joint>
  <link name="propeller_1">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/tmotor_1344_cw.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.012" radius="0.1651"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0142"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.693633550000001e-05" ixy="0.0" ixz="0.0" iyy="9.693633550000001e-05" iyz="0.0" izz="0.00019353187100000003"/>
    </inertial>
  </link>
  <joint name="arm_2_joint" type="tilt">
    <origin rpy="0 0 2.6179938779914944" xyz="-0.29401562458481695 0.16974999999999998 0.0305"/>
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="arm_2"/>
    <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
    <limit effort="1.6" lower="-3.141592653589793" upper="3.141592653589793" velocity="11.100294042683936"/>
  </joint>
  <link name="arm_2">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="144.803e-3"/>
      <origin rpy="0 0 0" xyz="-13.846e-3 -0.014e-3 -1.095e-3"/>
      <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
    </inertial>
  </link>
  <joint name="propeller_2_joint" type="thrust">
    <parent link="arm_2"/>
    <child link="propeller_2"/>
    <origin rpy="0 0 0" xyz="0 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="ccw"/>
  </joint>
  <link name="propeller_2">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/tmotor_1344_ccw.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.012" radius="0.1651"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0142"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.693633550000001e-05" ixy="0.0" ixz="0.0" iyy="9.693633550000001e-05" iyz="0.0" izz="0.00019353187100000003"/>
    </inertial>
  </link>
  <joint name="arm_3_joint" type="tilt">
    <origin rpy="0 0 3.6651914291880923" xyz="-0.29401562458481695 -0.16975000000000004 0.0305"/>
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="arm_3"/>
    <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
    <limit effort="1.6" lower="-3.141592653589793" upper="3.141592653589793" velocity="11.100294042683936"/>
  </joint>
  <link name="arm_3">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="144.803e-3"/>
      <origin rpy="0 0 0" xyz="-13.846e-3 -0.014e-3 -1.095e-3"/>
      <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
    </inertial>
  </link>
  <joint name="propeller_3_joint" type="thrust">
    <parent link="arm_3"/>
    <child link="propeller_3"/>
    <origin rpy="0 0 0" xyz="0 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="cw"/>
  </joint>
  <link name="propeller_3">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/tmotor_1344_cw.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.012" radius="0.1651"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0142"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.693633550000001e-05" ixy="0.0" ixz="0.0" iyy="9.693633550000001e-05" iyz="0.0" izz="0.00019353187100000003"/>
    </inertial>
  </link>
  <joint name="arm_4_joint" type="tilt">
    <origin rpy="0 0 4.71238898038469" xyz="-6.236513824657896e-17 -0.3395 0.0305"/>
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="arm_4"/>
    <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
    <limit effort="1.6" lower="-3.141592653589793" upper="3.141592653589793" velocity="11.100294042683936"/>
  </joint>
  <link name="arm_4">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="144.803e-3"/>
      <origin rpy="0 0 0" xyz="-13.846e-3 -0.014e-3 -1.095e-3"/>
      <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
    </inertial>
  </link>
  <joint name="propeller_4_joint" type="thrust">
    <parent link="arm_4"/>
    <child link="propeller_4"/>
    <origin rpy="0 0 0" xyz="0 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="ccw"/>
  </joint>
  <link name="propeller_4">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/tmotor_1344_ccw.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.012" radius="0.1651"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0142"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.693633550000001e-05" ixy="0.0" ixz="0.0" iyy="9.693633550000001e-05" iyz="0.0" izz="0.00019353187100000003"/>
    </inertial>
  </link>
  <joint name="arm_5_joint" type="tilt">
    <origin rpy="0 0 5.759586531581287" xyz="0.29401562458481684 -0.16975000000000015 0.0305"/>
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="arm_5"/>
    <!-- Dynamixel XC430-W150 @12V: https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ -->
    <limit effort="1.6" lower="-3.141592653589793" upper="3.141592653589793" velocity="11.100294042683936"/>
  </joint>
  <link name="arm_5">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/arm.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="144.803e-3"/>
      <origin rpy="0 0 0" xyz="-13.846e-3 -0.014e-3 -1.095e-3"/>
      <inertia ixx="30818e-9" ixy="183e-9" ixz="-804e-9" iyy="3.103e-4" iyz="-26.457e-9" izz="3.227e-4"/>
    </inertial>
  </link>
  <joint name="propeller_5_joint" type="thrust">
    <parent link="arm_5"/>
    <child link="propeller_5"/>
    <origin rpy="0 0 0" xyz="0 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="cw"/>
  </joint>
  <link name="propeller_5">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_voliro_like_description/meshes/tmotor_1344_cw.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.012" radius="0.1651"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0142"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="9.693633550000001e-05" ixy="0.0" ixz="0.0" iyy="9.693633550000001e-05" iyz="0.0" izz="0.00019353187100000003"/>
    </inertial>
  </link>
  <joint name="lg_base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="lg_base_link"/>
  </joint>
  <link name="lg_base_link">
    <!-- A very light point mass -->
    <inertial>
      <mass value="1e-6"/>
      <inertia ixx="1e-12" ixy="0" ixz="0" iyy="1e-12" iyz="0" izz="1e-12"/>
    </inertial>
  </link>
  <joint name="lg_left_top_joint" type="fixed">
    <origin rpy="0 0 0.0" xyz="0 0.0825 0"/>
    <parent link="lg_base_link"/>
    <child link="lg_left_top"/>
  </joint>
  <link name="lg_left_top">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/landing_gear/zhizicathy_electric_150/top.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/landing_gear/zhizicathy_electric_150/top.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0988"/>
      <origin rpy="0 0 0" xyz="0 0.0175 -0.0070"/>
      <inertia ixx="3.841e-5" ixy="0" ixz="0" iyy="5.358e-6" iyz="8.642e-7" izz="4.031e-5"/>
    </inertial>
  </link>
  <joint name="lg_left_bottom_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -0.0125"/>
    <parent link="lg_left_top"/>
    <child link="lg_left_bottom"/>
    <axis xyz="1 0 0"/>
    <limit effort="10" lower="-0.01" upper="0.9599310885968813" velocity="0.2181661564992912"/>
    <dynamics damping="0" friction="0.1"/>
  </joint>
  <link name="lg_left_bottom">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/landing_gear/zhizicathy_electric_150/bottom.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.2617993877991494 0 0" xyz="0 0.013 -0.085"/>
      <geometry>
        <cylinder length="0.18" radius="0.008"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 0" xyz="0 0.036 -0.17"/>
      <geometry>
        <box size="0.24 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0706"/>
      <origin rpy="0 0 0" xyz="0 0.029 -0.142"/>
      <inertia ixx="2.169e-4" ixy="0" ixz="0" iyy="4.320e-4" iyz="4.968e-5" izz="2.437e-4"/>
    </inertial>
  </link>
  <joint name="lg_right_top_joint" type="fixed">
    <origin rpy="0 0 3.141592653589793" xyz="0 -0.0825 0"/>
    <parent link="lg_base_link"/>
    <child link="lg_right_top"/>
  </joint>
  <link name="lg_right_top">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/landing_gear/zhizicathy_electric_150/top.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/landing_gear/zhizicathy_electric_150/top.stl" scale="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0988"/>
      <origin rpy="0 0 0" xyz="0 0.0175 -0.0070"/>
      <inertia ixx="3.841e-5" ixy="0" ixz="0" iyy="5.358e-6" iyz="8.642e-7" izz="4.031e-5"/>
    </inertial>
  </link>
  <joint name="lg_right_bottom_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -0.0125"/>
    <parent link="lg_right_top"/>
    <child link="lg_right_bottom"/>
    <axis xyz="1 0 0"/>
    <limit effort="10" lower="-0.01" upper="0.9599310885968813" velocity="0.2181661564992912"/>
    <dynamics damping="0" friction="0.1"/>
  </joint>
  <link name="lg_right_bottom">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_description/meshes/common/landing_gear/zhizicathy_electric_150/bottom.stl" scale="1 1 1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.2617993877991494 0 0" xyz="0 0.013 -0.085"/>
      <geometry>
        <cylinder length="0.18" radius="0.008"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 0" xyz="0 0.036 -0.17"/>
      <geometry>
        <box size="0.24 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.0706"/>
      <origin rpy="0 0 0" xyz="0 0.029 -0.142"/>
      <inertia ixx="2.169e-4" ixy="0" ixz="0" iyy="4.320e-4" iyz="4.968e-5" izz="2.437e-4"/>
    </inertial>
  </link>
</robot>
```

</details>
<br>

### Transformable Multicopter (<a href=https://github.com/TobasFlightControl/tobas_hydrus_description target="_blank">JSK Hydrus</a>)

This quadcopter has four `thrust` joints and three `revolute` joints.
Users can freely specify command values for the `revolute` joints.

![hydrus](../../assets/what_is_uadf/hydrus.png)

<details><summary>Before XACRO expansion</summary>

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="hydrus">

  <!-- basic kinematics model -->
  <xacro:include filename="$(find tobas_hydrus_description)/urdf/common.xacro"/>
  <xacro:include filename="$(find tobas_hydrus_description)/urdf/link.uadf"/>

  <xacro:hydrus_link links="4" self="1" rotor_direction="-1" with_battery="0"/>
  <xacro:hydrus_link links="4" self="2" rotor_direction="1" with_battery="0"/>
  <xacro:hydrus_link links="4" self="3" rotor_direction="-1" with_battery="0"/>
  <xacro:hydrus_link links="4" self="4" rotor_direction="1" with_battery="0"/>

  <!-- special battery arrangement -->
  <xacro:extra_module name="bat1" parent="link1" visible="1" model_url="package://tobas_hydrus_description/meshes/battery/Kypom-3000-6s.dae">
    <origin xyz="${link_length/2} 0.0 -0.048" rpy="0 0 0"/>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.4108"/>
      <inertia ixx="0.0001" iyy="0.0006" izz="0.0006" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </xacro:extra_module>

  <xacro:extra_module name="bat2" parent="link4" visible="1" model_url="package://tobas_hydrus_description/meshes/battery/Kypom-3000-6s.dae">
    <origin xyz="${link_length/2} 0.0 -0.048" rpy="0 0 0"/>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.4108"/>
      <inertia ixx="0.0001" iyy="0.0006" izz="0.0006" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </xacro:extra_module>

  <!-- onboard -->
  <!-- 1.  processor -->
  <!-- 1.1 flight controller -->
  <xacro:extra_module name="fc" parent="link2" visible="1" model_url="package://tobas_hydrus_description/meshes/ver2/flight_controller/spinal.dae">    <!-- same with intel euclid -->
    <origin xyz="${link_length / 2 + 0.2221} ${-4.4*0.001} 0.02135" rpy="0 0 0"/>
    <inertial>
      <mass value="0.05"/>
      <origin xyz="${13*0.001} ${4.4*0.001} ${-9*0.001}" rpy="0 0 0"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1.9999e-5"/>
    </inertial>
  </xacro:extra_module>
  <!-- end: flight controller -->

  <!-- 1.2 processor: jetson tx2 -->
  <!-- confirm the inertial parameter with real machine -->
  <xacro:extra_module name="pc" parent="link3" visible="1" model_url="package://tobas_hydrus_description/meshes/ver2/processor/jetson_tx2_unit.dae">
    <origin xyz="${link_length / 2 - 0.23775} 0.0 ${-0.01}" rpy="0 0 0"/>
    <inertial>
      <mass value="0.181"/>
      <origin xyz="${18 * 0.001} ${4 * 0.001} ${-18 * 0.001}" rpy="0 0 0"/>
      <inertia ixx="0.00007" ixy="0.0" ixz="0.0" iyy="0.00003" iyz="0.0" izz="0.00009"/>
    </inertial>
  </xacro:extra_module>
  <!-- end: processor: jetson tx2 -->

  <!-- 2.  sensor -->
  <!-- 2.1 leddar one -->
  <xacro:extra_module name="leddarone" parent="link3" visible="1" model_url="package://tobas_hydrus_description/meshes/sensor/leddar_one_tx2_attached_mode.dae">
    <origin xyz="${link_length/2 - 0.1855} 0.0 -0.0626" rpy="${pi} 0 0"/>
    <inertial>
      <origin xyz="-0.008 0.0 0.0" rpy="0 0 0"/>
      <mass value="0.025"/>
      <inertia ixx="0.00001" iyy="0.000006" izz="0.00001" ixy="0.000000" ixz="0.000000" iyz="0.000000"/>
    </inertial>
  </xacro:extra_module>
  <!-- end: leddar one -->

  <!-- 2.2 gps -->
  <xacro:extra_module name="gps" parent="link2" visible="1" model_url="package://tobas_hydrus_description/meshes/sensor/gps_ublox_m8n.dae">
    <origin xyz="${link_length/2 + 0.1925} 0.0 0.152" rpy="0 0 0"/>
    <inertial>
      <origin xyz="0.000000 0.000000 -0.013" rpy="0 0 0"/>
      <mass value="0.042"/>
      <inertia ixx="0.00006" iyy="0.00006" izz="0.000007" ixy="0.000000" ixz="0.000000" iyz="0.000000"/>
    </inertial>
  </xacro:extra_module>
  <xacro:extra_module name="magnet" parent="gps">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertial>
      <mass value="0.00001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1.9999e-6"/>
    </inertial>
  </xacro:extra_module>
  <!-- end: gps -->

</robot>
```

</details>

<details><summary>After XACRO expansion</summary>

```xml
<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from ./src/tobas_uadf_examples/tobas_hydrus_description/urdf/hydrus.uadf | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="hydrus">
  <!-- general attribute -->
  <baselink name="fc"/>
  <thrust_link name="thrust"/>
  <!-- dynamics -->
  <m_f_rate value="-0.0172"/>
  <!-- [N] -->
  <!-- link -->
  <link name="link1">
    <inertial>
      <origin rpy="0 0 0" xyz="0.357 0.0 0.018"/>
      <mass value="0.58712"/>
      <inertia ixx="0.00179" ixy="0.00000" ixz="0.00067" iyy="0.01605" iyz="0.00000" izz="0.01714"/>
    </inertial>
    <!-- for fcl in planning -->
    <collision>
      <origin rpy="0 0 0" xyz="0.3 0 -0.08575"/>
      <geometry>
        <box size="0.6 0.05 0.1715"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0.3 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/link/link1.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="link1">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <!-- rotor -->
  <joint name="rotor1" type="thrust">
    <parent link="link1"/>
    <child link="thrust1"/>
    <origin rpy="0 0 0" xyz="0.3 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="cw"/>
  </joint>
  <link name="thrust1">
    <!-- visual & collisiont -->
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1.9999e-5"/>
    </inertial>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.2025"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.032"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/propeller/T-motor-CF-14inch-CW.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="thrust1">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <joint name="link12leg1" type="fixed">
    <parent link="link1"/>
    <child link="leg1"/>
    <origin rpy="0 0 0" xyz="0.041159999999999974 0 0"/>
  </joint>
  <link name="leg1">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0005 -0.09411"/>
      <mass value="0.0471"/>
      <inertia ixx="0.000251" ixy="0.000000" ixz="0.000000" iyy="0.000242" iyz="0.000000" izz="0.000011"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/leg/end_leg.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <limit effort="10.0" lower="-1.5707963267948966" upper="1.5707963267948966" velocity="0.5"/>
    <parent link="link1"/>
    <child link="link2"/>
    <origin rpy="0 0 0" xyz="0.6 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.9" friction="0.05"/>
  </joint>
  <link name="root"/>
  <joint name="root_joint" type="fixed">
    <parent link="root"/>
    <child link="link1"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <transmission name="joint_tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="servo{self}">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <gazebo reference="link1">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>
  <!-- link -->
  <link name="link2">
    <inertial>
      <origin rpy="0 0 0" xyz="0.3 0.0 0.01287"/>
      <mass value="0.55744"/>
      <inertia ixx="0.00171" ixy="0.00000" ixz="0.00001" iyy="0.01727" iyz="0.00000" izz="0.01844"/>
    </inertial>
    <!-- for fcl in planning -->
    <collision>
      <origin rpy="0 0 0" xyz="0.3 0 -0.08575"/>
      <geometry>
        <box size="0.6 0.05 0.1715"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0.3 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/link/link2.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="link2">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <!-- rotor -->
  <joint name="rotor2" type="thrust">
    <parent link="link2"/>
    <child link="thrust2"/>
    <origin rpy="0 0 0" xyz="0.3 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="ccw"/>
  </joint>
  <link name="thrust2">
    <!-- visual & collisiont -->
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1.9999e-5"/>
    </inertial>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.2025"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.032"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/propeller/T-motor-CF-14inch-CCW.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="thrust2">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <joint name="link22leg2" type="fixed">
    <parent link="link2"/>
    <child link="leg2"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="leg2">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0005 -0.09521"/>
      <mass value="0.0471"/>
      <inertia ixx="0.000251" ixy="0.000000" ixz="0.000000" iyy="0.000242" iyz="0.000000" izz="0.000011"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/leg/joint_leg.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint2" type="revolute">
    <limit effort="10.0" lower="-1.5707963267948966" upper="1.5707963267948966" velocity="0.5"/>
    <parent link="link2"/>
    <child link="link3"/>
    <origin rpy="0 0 0" xyz="0.6 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.9" friction="0.05"/>
  </joint>
  <transmission name="joint_tran2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="servo{self}">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <gazebo reference="link2">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>
  <!-- link -->
  <link name="link3">
    <inertial>
      <origin rpy="0 0 0" xyz="0.2683 0.0 0.01836"/>
      <mass value="0.63453"/>
      <inertia ixx="0.00182" ixy="0.00000" ixz="-0.00077" iyy="0.02232" iyz="0.00000" izz="0.02254"/>
    </inertial>
    <!-- for fcl in planning -->
    <collision>
      <origin rpy="0 0 0" xyz="0.3 0 -0.08575"/>
      <geometry>
        <box size="0.6 0.05 0.1715"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0.3 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/link/link3.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="link3">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <!-- rotor -->
  <joint name="rotor3" type="thrust">
    <parent link="link3"/>
    <child link="thrust3"/>
    <origin rpy="0 0 0" xyz="0.3 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="cw"/>
  </joint>
  <link name="thrust3">
    <!-- visual & collisiont -->
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1.9999e-5"/>
    </inertial>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.2025"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.032"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/propeller/T-motor-CF-14inch-CW.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="thrust3">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <joint name="link32leg3" type="fixed">
    <parent link="link3"/>
    <child link="leg3"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="leg3">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0005 -0.09521"/>
      <mass value="0.0471"/>
      <inertia ixx="0.000251" ixy="0.000000" ixz="0.000000" iyy="0.000242" iyz="0.000000" izz="0.000011"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/leg/joint_leg.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint3" type="revolute">
    <limit effort="10.0" lower="-1.5707963267948966" upper="1.5707963267948966" velocity="0.5"/>
    <parent link="link3"/>
    <child link="link4"/>
    <origin rpy="0 0 0" xyz="0.6 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.9" friction="0.05"/>
  </joint>
  <transmission name="joint_tran3">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint3">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="servo{self}">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <gazebo reference="link3">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>
  <!-- link -->
  <link name="link4">
    <inertial>
      <origin rpy="0 0 0" xyz="0.242 0.0 0.0188"/>
      <mass value="0.590"/>
      <!-- +DCDC -->
      <inertia ixx="0.00179" ixy="0.00000" ixz="-0.00067" iyy="0.01581" iyz="0.00000" izz="0.01692"/>
    </inertial>
    <!-- for fcl in planning -->
    <collision>
      <origin rpy="0 0 0" xyz="0.3 0 -0.08575"/>
      <geometry>
        <box size="0.6 0.05 0.1715"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0.3 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/link/link4.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="link4">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <!-- rotor -->
  <joint name="rotor4" type="thrust">
    <parent link="link4"/>
    <child link="thrust4"/>
    <origin rpy="0 0 0" xyz="0.3 0 0.013"/>
    <axis xyz="0 0 1"/>
    <direction value="ccw"/>
  </joint>
  <link name="thrust4">
    <!-- visual & collisiont -->
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.0001"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1.9999e-5"/>
    </inertial>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.2025"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.032"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/propeller/T-motor-CF-14inch-CCW.dae"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="thrust4">
    <dampingFactor>0.00</dampingFactor>
  </gazebo>
  <joint name="link42leg4" type="fixed">
    <parent link="link4"/>
    <child link="leg4"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="leg4">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0005 -0.09521"/>
      <mass value="0.0471"/>
      <inertia ixx="0.000251" ixy="0.000000" ixz="0.000000" iyy="0.000242" iyz="0.000000" izz="0.000011"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/leg/joint_leg.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="link42leg5" type="fixed">
    <parent link="link4"/>
    <child link="leg5"/>
    <origin rpy="0 0 0" xyz="0.55884 0 0"/>
  </joint>
  <link name="leg5">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0005 -0.09411"/>
      <mass value="0.0471"/>
      <inertia ixx="0.000251" ixy="0.000000" ixz="0.000000" iyy="0.000242" iyz="0.000000" izz="0.000011"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/leg/end_leg.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="link4">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>
  <joint name="link12bat1" type="fixed">
    <parent link="link1"/>
    <child link="bat1"/>
    <origin rpy="0 0 0" xyz="0.3 0.0 -0.048"/>
  </joint>
  <link name="bat1">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.4108"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0006" iyz="0.0" izz="0.0006"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/battery/Kypom-3000-6s.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="link42bat2" type="fixed">
    <parent link="link4"/>
    <child link="bat2"/>
    <origin rpy="0 0 0" xyz="0.3 0.0 -0.048"/>
  </joint>
  <link name="bat2">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.4108"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0006" iyz="0.0" izz="0.0006"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/battery/Kypom-3000-6s.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="link22fc" type="fixed">
    <parent link="link2"/>
    <child link="fc"/>
    <origin rpy="0 0 0" xyz="0.5221 -0.0044 0.02135"/>
  </joint>
  <link name="fc">
    <inertial>
      <mass value="0.05"/>
      <origin rpy="0 0 0" xyz="0.013000000000000001 0.0044 -0.009000000000000001"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1.9999e-5"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/flight_controller/spinal.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <!-- end: flight controller -->
  <joint name="link32pc" type="fixed">
    <parent link="link3"/>
    <child link="pc"/>
    <origin rpy="0 0 0" xyz="0.06225 0.0 -0.01"/>
  </joint>
  <link name="pc">
    <inertial>
      <mass value="0.181"/>
      <origin rpy="0 0 0" xyz="0.018000000000000002 0.004 -0.018000000000000002"/>
      <inertia ixx="0.00007" ixy="0.0" ixz="0.0" iyy="0.00003" iyz="0.0" izz="0.00009"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/ver2/processor/jetson_tx2_unit.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <!-- end: processor: jetson tx2 -->
  <joint name="link32leddarone" type="fixed">
    <parent link="link3"/>
    <child link="leddarone"/>
    <origin rpy="3.141592653589793 0 0" xyz="0.11449999999999999 0.0 -0.0626"/>
  </joint>
  <link name="leddarone">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.008 0.0 0.0"/>
      <mass value="0.025"/>
      <inertia ixx="0.00001" ixy="0.000000" ixz="0.000000" iyy="0.000006" iyz="0.000000" izz="0.00001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/sensor/leddar_one_tx2_attached_mode.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <!-- end: leddar one -->
  <joint name="link22gps" type="fixed">
    <parent link="link2"/>
    <child link="gps"/>
    <origin rpy="0 0 0" xyz="0.4925 0.0 0.152"/>
  </joint>
  <link name="gps">
    <inertial>
      <origin rpy="0 0 0" xyz="0.000000 0.000000 -0.013"/>
      <mass value="0.042"/>
      <inertia ixx="0.00006" ixy="0.000000" ixz="0.000000" iyy="0.00006" iyz="0.000000" izz="0.000007"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://tobas_hydrus_description/meshes/sensor/gps_ublox_m8n.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="gps2magnet" type="fixed">
    <parent link="gps"/>
    <child link="magnet"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="magnet">
    <inertial>
      <mass value="0.00001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1.9999e-6"/>
    </inertial>
  </link>
  <!-- end: gps -->
</robot>
```

</details>
<br>
