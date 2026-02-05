---
sidebar_position: 1
title: "Humanoid Modeling with URDF"
---

# Humanoid Modeling with URDF

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamentals of Unified Robot Description Format (URDF)
- Create complex humanoid robot models using URDF
- Define joints, links, and sensors for humanoid robots
- Implement kinematic chains for humanoid locomotion
- Validate and simulate humanoid models in ROS 2

## Introduction

Humanoid robots represent one of the most challenging and fascinating areas of robotics. Unlike wheeled or manipulator robots, humanoid robots must deal with complex balance, locomotion, and interaction with human environments. The Unified Robot Description Format (URDF) is ROS's native format for representing robots, and it provides the necessary tools to model the complex kinematics and dynamics of humanoid robots.

This chapter will guide you through the process of creating realistic humanoid robot models using URDF, with a focus on proper joint configurations, link properties, and sensor placement that are essential for humanoid robotics applications.

## What is URDF?

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robots. It defines the physical and visual properties of a robot, including:

- **Links**: Rigid bodies with physical properties (mass, inertia, visual appearance)
- **Joints**: Connections between links that define motion constraints
- **Materials**: Visual appearance properties
- **Gazebo plugins**: Simulation-specific extensions
- **Transmission interfaces**: Actuator control definitions

URDF is not a complete description of a robot - it's primarily concerned with rigid-body transforms and some physical properties. It doesn't describe dynamic properties like friction coefficients or spring constants.

## Basic URDF Structure

A basic URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
  </joint>

  <!-- Define additional links -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>
</robot>
```

## URDF Elements for Humanoid Robots

### Links

For humanoid robots, typical links include:

- **base_link**: Usually the pelvis or torso
- **Head**: Head link with sensors
- **Torso**: Torso links (sometimes multiple for spine flexibility)
- **Arms**: Shoulder, upper arm, forearm, wrist, hand links
- **Legs**: Hip, thigh, shin, foot links
- **Additional**: Sensors, cameras, etc.

### Joints

Humanoid robots require various joint types:

- **Revolute**: Rotational joints with limited range (elbows, knees)
- **Continuous**: Rotational joints without limits (shoulders, hips)
- **Fixed**: Non-moving joints (attachment of sensors)
- **Prismatic**: Linear sliding joints (less common in humanoids)

## Humanoid Kinematic Chain Structure

Humanoid robots typically follow this kinematic structure:

```
base_link (pelvis)
├── torso
│   ├── head
│   ├── left_shoulder
│   │   ├── left_upper_arm
│   │   │   ├── left_forearm
│   │   │   │   ├── left_wrist
│   │   │   │   │   └── left_hand
│   │   └── ...
│   ├── right_shoulder
│   │   ├── right_upper_arm
│   │   │   ├── right_forearm
│   │   │   │   ├── right_wrist
│   │   │   │   │   └── right_hand
│   │   └── ...
│   ├── left_hip
│   │   ├── left_thigh
│   │   │   ├── left_shin
│   │   │   │   └── left_foot
│   │   └── ...
│   └── right_hip
│       ├── right_thigh
│       │   ├── right_shin
│       │   │   └── right_foot
│       └── ...
```

## Detailed Humanoid URDF Example

Let's create a simplified humanoid model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Materials -->
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <!-- Base Link (Pelvis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.25 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.25 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Similar structure for right arm, legs, etc. -->
</robot>
```

## URDF Best Practices for Humanoids

### 1. Proper Mass and Inertia Values

Accurate mass and inertia properties are crucial for simulation:

```xml
<!-- Example of proper inertial calculation -->
<inertial>
  <mass value="1.5"/>
  <!-- For a cylinder: Ixx = Iyy = m*(3*r² + h²)/12, Izz = m*r²/2 -->
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003"/>
</inertial>
```

### 2. Joint Limits and Safety

Define realistic joint limits to prevent damage:

```xml
<joint name="knee_joint" type="revolute">
  <parent link="thigh"/>
  <child link="shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <!-- Human knee can bend ~150 degrees max -->
  <limit lower="0" upper="2.6" effort="200" velocity="2"/>
</joint>
```

### 3. Collision Avoidance

Consider collision shapes carefully:

```xml
<link name="upper_arm">
  <collision>
    <!-- Use bounding box rather than exact shape for efficiency -->
    <geometry>
      <capsule radius="0.06" length="0.28"/>
    </geometry>
  </collision>
</link>
```

## Advanced URDF Features for Humanoids

### 1. Mimic Joints

For symmetrical movements:

```xml
<joint name="right_elbow_mirror" type="revolute">
  <parent link="right_upper_arm"/>
  <child link="right_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <mimic joint="left_elbow_joint" multiplier="1" offset="0"/>
</joint>
```

### 2. Gazebo Plugins

For simulation enhancements:

```xml
<gazebo reference="base_link">
  <material>Gazebo/White</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="control_plugin" filename="libgazebo_ros_control.so">
    <robotNamespace>/simple_humanoid</robotNamespace>
  </plugin>
</gazebo>
```

## Validation and Testing

### 1. URDF Validation

Use ROS tools to validate your URDF:

```bash
# Check for parsing errors
check_urdf /path/to/robot.urdf

# Generate graph of robot structure
urdf_to_graphiz /path/to/robot.urdf
```

### 2. Visualization

Visualize your robot in RViz:

```xml
<!-- Add this to your URDF for better visualization -->
<transmission name="simple_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Common Pitfalls and Solutions

### 1. Floating Point Precision

Use appropriate precision in URDF files:

```xml
<!-- Good -->
<origin xyz="0.0 0.0 0.5" rpy="0.0 0.0 0.0"/>

<!-- Avoid -->
<origin xyz="0.000000000001 0 -0.000000000002" rpy="0 0 0"/>
```

### 2. Joint Axis Alignment

Ensure joint axes are correctly aligned with intended motion:

```xml
<!-- For a hinge joint that rotates around Y-axis -->
<axis xyz="0 1 0"/>
```

### 3. Link Overlap Prevention

Avoid overlapping collision geometries that can cause simulation instability.

## Practice Exercise

Create a URDF model of a simple humanoid robot with:
1. A base link (pelvis)
2. Torso with head
3. Two arms with shoulders and elbows
4. Two legs with hips and knees
5. Proper joint limits and masses
6. Basic collision and visual properties

Validate your model using ROS tools and visualize it in RViz.

## Summary

Creating humanoid robot models with URDF requires attention to anatomical accuracy, proper joint definitions, and realistic physical properties. By following best practices and using advanced features like mimic joints and Gazebo plugins, you can create sophisticated humanoid models suitable for simulation and control in ROS 2. Remember to validate your models and consider the specific requirements of humanoid robotics, such as balance and locomotion capabilities.