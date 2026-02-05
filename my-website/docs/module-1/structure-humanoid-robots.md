---
sidebar_position: 3
title: "Structure of Humanoid Robots"
---

# Structure of Humanoid Robots

## Learning Objectives

After completing this section, you will be able to:
- Design hierarchical structures for humanoid robot models
- Implement kinematic chains for humanoid locomotion and manipulation
- Create proper coordinate frames for humanoid robots
- Understand the relationship between biological and mechanical structures
- Apply best practices for organizing complex humanoid URDF models

## Introduction

The structure of humanoid robots encompasses both the mechanical design that mimics human form and the computational representation that enables control and simulation. Creating an effective humanoid robot structure requires understanding both biological inspiration and engineering constraints. This section explores how to translate the complex structure of the human body into a computational model suitable for ROS 2 applications.

## Biological Inspiration vs. Mechanical Reality

### Human Body Structure

The human body is incredibly complex with approximately 206 bones, 600+ muscles, and numerous joints of different types. For humanoid robots, we must make simplifications while preserving essential characteristics:

**Key Human Structural Features:**
- Bilateral symmetry
- Central axis (spine) with limbs attached
- Multiple degrees of freedom per limb
- Redundant kinematic chains
- Distributed sensing and actuation

### Mechanical Simplification Strategies

**1. Degree of Freedom Reduction:**
- Human: ~244 DOF (theoretical)
- Typical humanoid: 20-50 DOF
- Focus on essential movements for intended tasks

**2. Joint Approximation:**
- Replace ball-and-socket joints with combinations of revolute joints
- Simplify complex joint mechanics to basic kinematic pairs
- Maintain range of motion within biological limits

**3. Segment Combination:**
- Combine multiple bones into single rigid links
- Group segments that move together

## Hierarchical Structure Organization

### Root-First Approach

Start from the robot's base and build upward:

```xml
<robot name="humanoid_robot">
  <!-- Base link (pelvis/root) -->
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
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
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

  <!-- Continue with head, arms, legs -->
</robot>
```

### Modular Organization

Organize the robot structure into logical modules:

```xml
<!-- Include separate files for different modules -->
<xacro:include filename="$(find humanoid_description)/urdf/head.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/left_arm.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/right_arm.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/left_leg.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/right_leg.urdf.xacro"/>
```

## Kinematic Chain Design for Humanoids

### Forward Kinematic Chains

Humanoid robots typically have multiple kinematic chains:

**1. Right Arm Chain:**
```
base_link → torso → right_shoulder → right_upper_arm → right_forearm → right_hand
```

**2. Left Arm Chain:**
```
base_link → torso → left_shoulder → left_upper_arm → left_forearm → left_hand
```

**3. Right Leg Chain:**
```
base_link → right_hip → right_thigh → right_shin → right_foot
```

**4. Left Leg Chain:**
```
base_link → left_hip → left_thigh → left_shin → left_foot
```

### Kinematic Considerations

**Redundancy:** Humanoid arms often have more DOFs than required for end-effector positioning, allowing for posture optimization.

**Workspace:** Ensure each chain can reach its intended workspace without collisions.

**Balance:** The center of mass must remain within the support polygon for stable stance.

### Example Full Arm Chain:

```xml
<!-- Right Arm Kinematic Chain -->
<joint name="right_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="right_upper_arm"/>
  <origin xyz="0.1 0.1 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>

<joint name="right_shoulder_roll" type="revolute">
  <parent link="right_upper_arm"/>
  <child link="right_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.356" upper="0" effort="100" velocity="2"/>
</joint>

<joint name="right_elbow" type="revolute">
  <parent link="right_lower_arm"/>
  <child link="right_hand"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.356" upper="0" effort="100" velocity="2"/>
</joint>
```

## Coordinate Frame Systems

### Base Frame Selection

The base frame choice significantly impacts control complexity:

**Option 1: Pelvis Frame**
- Advantages: Natural zero-moment point reference
- Disadvantages: Moves during locomotion

**Option 2: Torso Frame**
- Advantages: More stable during walking
- Disadvantages: Offset from mechanical base

**Option 3: World Frame**
- Advantages: Fixed reference
- Disadvantages: Continuous transformation updates

### Frame Naming Convention

Use consistent TF naming for humanoid robots:

```
base_link (robot origin)
├── odom (world reference)
├── map (global reference)
├── torso (torso frame)
├── head (head frame)
├── left_shoulder
├── left_elbow
├── left_wrist
├── right_shoulder
├── right_elbow
├── right_wrist
├── left_hip
├── left_knee
├── left_ankle
├── right_hip
├── right_knee
└── right_ankle
```

### Sensor Frame Integration

Properly attach sensor frames:

```xml
<!-- IMU in pelvis for balance -->
<joint name="imu_frame_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_frame"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
<link name="imu_frame"/>

<!-- Camera in head for vision -->
<joint name="head_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="head_camera_frame"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
<link name="head_camera_frame"/>
```

## Balance and Stability Considerations

### Center of Mass (CoM)

For stable humanoid locomotion:

```xml
<!-- Example CoM calculation in URDF -->
<!-- The overall CoM should be within the support polygon -->
<!-- This affects the inertial properties of the base link -->
<inertial>
  <mass value="30"/>  <!-- Total robot mass -->
  <origin xyz="0 0 0.5" rpy="0 0 0"/>  <!-- CoM position -->
  <inertia ixx="1.5" ixy="0" ixz="0" iyy="1.5" iyz="0" izz="1.0"/>
</inertial>
```

### Zero-Moment Point (ZMP)

Critical for walking stability:

- ZMP should remain within the support polygon
- Achieved through coordinated joint motion
- Requires precise mass distribution modeling

### Support Polygon

For bipedal robots:

- When standing: polygon between feet
- When stepping: point at stance foot
- Robot must maintain CoM projection within this polygon

## Manipulation Structure Design

### Anthropomorphic Design Principles

**Reach Envelope:** Arms should reach areas humans typically interact with:
- Forward: ~1.2x height
- Sideways: ~0.8x height
- Downward: ~0.2x height from hip

**Dexterity vs. Strength Trade-off:**
- More DOFs increase dexterity but complexity
- Fewer DOFs increase strength and reliability

### Hand/End-Effector Considerations

Simple grippers vs. anthropomorphic hands:

```xml
<!-- Simple parallel jaw gripper -->
<joint name="gripper_joint" type="prismatic">
  <parent link="wrist"/>
  <child link="gripper_left"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="0.05" effort="50" velocity="1"/>
</joint>

<!-- Attach gripper to wrist -->
<link name="gripper_left">
  <visual>
    <geometry>
      <box size="0.05 0.01 0.02"/>
    </geometry>
    <material name="gray"/>
  </visual>
</link>
```

## Locomotion Structure Design

### Bipedal Walking Considerations

**Leg Structure:**
- 6+ DOFs per leg for stable walking
- Adequate ground clearance
- Proper foot design for contact

**Foot Design:**
- Sufficient size for stability
- Proper contact points
- Sensor integration for ground contact

```xml
<!-- Example foot structure -->
<link name="left_foot">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
  </inertial>
</link>

<joint name="left_ankle" type="revolute">
  <parent link="left_shin"/>
  <child link="left_foot"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Pitch axis -->
  <limit lower="-0.5" upper="0.5" effort="100" velocity="2"/>
</joint>
```

## Best Practices for Complex URDF Models

### 1. Use Xacro for Modularity

Xacro (XML Macros) allows parameterized and modular URDF design:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="humanoid_link" params="name mass xyz rpy size material">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
        <material name="${material}"/>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create links -->
  <xacro:humanoid_link name="torso" mass="8" xyz="0 0 0" rpy="0 0 0" size="0.2 0.2 0.4" material="white"/>

</robot>
```

### 2. Parameter Files

Store dimensions and properties in separate parameter files:

```xml
<!-- dimensions.xacro -->
<xacro:property name="torso_height" value="0.4" />
<xacro:property name="upper_arm_length" value="0.3" />
<xacro:property name="lower_arm_length" value="0.25" />
<xacro:property name="leg_length" value="0.9" />
```

### 3. Consistent Naming

Use a consistent naming scheme:

```xml
<!-- Good naming convention -->
<link name="left_upper_arm_link"/>
<joint name="left_shoulder_pitch_joint"/>
<transmission name="left_elbow_transmission"/>

<!-- Bad naming convention -->
<link name="arm1"/>
<joint name="j1"/>
<transmission name="t1"/>
```

### 4. Documentation

Document your structure clearly:

```xml
<!-- Left Arm Structure -->
<!-- DOF: 7 -->
<!-- Joints: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_yaw, wrist_roll -->
<!-- Links: left_upper_arm, left_forearm, left_hand -->
```

## Model Validation

### Structural Validation

Check for common structural issues:

```bash
# Check for parsing errors
check_urdf humanoid_model.urdf

# Generate kinematic graph
urdf_to_graphviz humanoid_model.urdf
ev humanoid_model.pdf  # View the kinematic tree
```

### Physical Property Validation

Verify mass properties are realistic:

```xml
<!-- Total mass should be reasonable -->
<!-- Individual link masses should make sense -->
<!-- Center of mass should be in reasonable location -->
```

### Kinematic Validation

Test reach and workspace:

```bash
# Use MoveIt! to test kinematic solvers
# Verify no impossible configurations
# Check for singularities
```

## Advanced Structural Features

### Flexible Spine

For more human-like movement:

```xml
<!-- Multi-segment spine -->
<joint name="spine_lower" type="revolute">
  <parent link="base_link"/>
  <child link="spine_mid"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.3" upper="0.3" effort="200" velocity="1"/>
</joint>

<joint name="spine_upper" type="revolute">
  <parent link="spine_mid"/>
  <child link="spine_top"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.3" upper="0.3" effort="200" velocity="1"/>
</joint>
```

### Variable Stiffness

Model muscle-like behavior:

```xml
<!-- Use joint dynamics to simulate muscle properties -->
<joint name="compliant_joint" type="revolute">
  <dynamics damping="10.0" friction="5.0"/>
  <!-- Higher damping = more compliant -->
</joint>
```

## Practice Exercise

Design a complete humanoid robot structure with:
1. A base link representing the pelvis
2. A segmented torso (optional flexibility)
3. Two arms with 7 DOFs each
4. Two legs with 6 DOFs each
5. A head with neck joints
6. Proper coordinate frames for all components
7. Mass properties that result in a realistic total robot weight (~50-100kg)
8. A simple hand design for each arm

Validate your model using URDF tools and visualize it in RViz.

## Summary

Creating the structure of humanoid robots requires balancing biological inspiration with engineering constraints. The hierarchical organization of links and joints, proper coordinate frame systems, and attention to balance and stability considerations are crucial for effective humanoid robot design. By following best practices for URDF organization and validation, you can create robust models suitable for both simulation and real-world control applications in ROS 2.