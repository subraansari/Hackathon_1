---
sidebar_position: 2
title: "URDF Links, Joints, and Sensors"
---

# URDF Links, Joints, and Sensors

## Learning Objectives

After completing this section, you will be able to:
- Define and configure URDF links with proper physical properties
- Implement various joint types with appropriate limits and dynamics
- Integrate sensors into humanoid robot models
- Understand the relationship between URDF elements and robot physics
- Validate link and joint configurations for humanoid applications

## Introduction

In the realm of humanoid robotics, the accurate representation of physical components is crucial for both simulation and real-world control. URDF (Unified Robot Description Format) provides the foundation for describing the mechanical structure of robots, with links, joints, and sensors forming the core elements. For humanoid robots, these elements must accurately represent the complex kinematic structure of the human body to enable realistic simulation and effective control.

This section will explore the detailed implementation of these elements, focusing on the specific requirements of humanoid robots.

## URDF Links

Links represent the rigid bodies of a robot. In humanoid robots, links correspond to body segments such as the torso, limbs, and individual bones.

### Link Structure

A basic link definition includes visual, collision, and inertial properties:

```xml
<link name="link_name">
  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="link_color"/>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Visual Properties

Visual elements define how the robot appears in visualization tools like RViz and simulation environments:

```xml
<visual>
  <!-- Origin defines the pose of the visual element relative to the link frame -->
  <origin xyz="0.05 0 0" rpy="0 0 1.57"/>

  <!-- Geometry defines the shape -->
  <geometry>
    <!-- Options: box, cylinder, sphere, mesh -->
    <cylinder radius="0.05" length="0.3"/>
  </geometry>

  <!-- Material defines the appearance -->
  <material name="blue_material"/>
</visual>
```

#### Geometry Types for Humanoid Robots:

- **Box**: Simple rectangular shapes for torso segments
- **Cylinder**: Ideal for limbs and joints
- **Sphere**: Good for head or joint representations
- **Mesh**: Complex shapes imported from CAD models

### Collision Properties

Collision elements define the shapes used for collision detection and physics simulation:

```xml
<collision>
  <!-- Often similar to visual but can be simplified for performance -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Simplified geometry for better performance -->
    <capsule radius="0.05" length="0.3"/>
  </geometry>
</collision>
```

#### Collision Optimization for Humanoids:

For humanoid robots, consider using:
- **Capsules** for limbs (better performance than cylinders)
- **Boxes** for torso and feet
- **Spheres** for heads and simple joints
- **Convex hulls** for complex shapes when precision is needed

### Inertial Properties

Inertial properties are crucial for accurate physics simulation, especially important for humanoid robots that must maintain balance:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.5"/>  <!-- Mass in kg -->
  <!-- Inertia tensor (symmetric, so only 6 values needed) -->
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
</inertial>
```

#### Calculating Inertial Properties:

For common shapes:

**Box** (width w, depth d, height h, mass m):
```
ixx = m * (h*h + d*d) / 12
iyy = m * (w*w + h*h) / 12
izz = m * (w*w + d*d) / 12
```

**Cylinder** (radius r, length l, mass m):
```
ixx = m * (3*r*r + l*l) / 12
iyy = m * (3*r*r + l*l) / 12
izz = m * r*r / 2
```

**Sphere** (radius r, mass m):
```
ixx = iyy = izz = 0.4 * m * r*r
```

### Link Naming Conventions for Humanoids

Adopt consistent naming for humanoid robots:

```
base_link (pelvis/root)
├── torso
├── head
├── left_upper_arm
├── left_lower_arm
├── left_hand
├── right_upper_arm
├── right_lower_arm
├── right_hand
├── left_thigh
├── left_shin
├── left_foot
├── right_thigh
├── right_shin
└── right_foot
```

## URDF Joints

Joints define the connections between links and constrain their relative motion. For humanoid robots, joints must accurately represent human joint capabilities.

### Joint Structure

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Joint Types for Humanoid Robots

#### 1. Fixed Joints

Used for permanent connections with no relative motion:

```xml
<joint name="sensor_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

#### 2. Revolute Joints

Rotational joints with limited range (most humanoid joints):

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Rotation around X-axis -->
  <limit lower="-2.356" upper="0" effort="50" velocity="2"/>  <!-- -135° to 0° -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

#### 3. Continuous Joints

Rotational joints without limits (shoulders, hips):

```xml
<joint name="shoulder_yaw" type="continuous">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y-axis -->
  <dynamics damping="0.3" friction="0.05"/>
</joint>
```

#### 4. Prismatic Joints

Linear sliding joints (less common in humanoids):

```xml
<joint name="linear_joint" type="prismatic">
  <parent link="base"/>
  <child link="slide_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.1" effort="100" velocity="0.5"/>
</joint>
```

### Joint Limits for Human Anatomy

Realistic joint limits based on human anatomy:

**Shoulder** (ball-and-socket - approximated with 2-3 revolute joints):
- Pitch: -90° to 90°
- Yaw: -90° to 90°
- Roll: -120° to 120°

**Elbow** (hinge):
- Flexion: -150° to 0° (allowing slight hyperextension)

**Hip** (ball-and-socket - approximated):
- Pitch: -30° to 70° (flexion/extension)
- Yaw: -20° to 20° (rotation)
- Roll: -30° to 30° (abduction/adduction)

**Knee** (hinge):
- Flexion: 0° to 150°

**Ankle**:
- Pitch: -30° to 30° (dorsiflexion/plantarflexion)
- Roll: -20° to 20° (inversion/eversion)

### Joint Dynamics

Damping and friction parameters affect simulation realism:

```xml
<dynamics damping="0.1" friction="0.01"/>
```

- **Damping**: Resistance to motion (higher values = more resistance)
- **Friction**: Resistance to initiate motion

## Sensors in Humanoid Robots

Sensors are critical for humanoid robots to perceive their environment and their own state.

### IMU Sensors

IMUs (Inertial Measurement Units) are essential for balance:

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-06" ixy="0" ixz="0" iyy="1e-06" iyz="0" izz="1e-06"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for IMU simulation -->
<gazebo reference="imu_link">
  <gravity>true</gravity>
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Force/Torque Sensors

FT sensors in humanoid feet for balance:

```xml
<gazebo reference="left_foot">
  <sensor name="left_foot_ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

### Camera Sensors

Vision systems for perception:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Advanced Joint Configurations

### Mimic Joints

For symmetric movements:

```xml
<joint name="right_elbow" type="revolute">
  <parent link="right_upper_arm"/>
  <child link="right_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="0" effort="50" velocity="2"/>
  <!-- Mirror the left elbow motion -->
  <mimic joint="left_elbow" multiplier="1" offset="0"/>
</joint>
```

### Transmission Interfaces

Connect joints to actuators:

```xml
<transmission name="elbow_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="elbow_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="elbow_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Validation Techniques

### Checking Joint Limits

Verify that joint limits are appropriate:

```bash
# Check the robot's kinematic chain
ros2 run tf2_tools view_frames
```

### Mass Property Validation

Ensure realistic mass distribution:

- Head: ~5-8% of total body weight
- Arms: ~5-6% each
- Legs: ~16-18% each
- Torso: ~50% of total body weight

### Visualization

Use RViz to verify the model:

```bash
ros2 run rviz2 rviz2
```

## Common Issues and Solutions

### 1. Floating Links

All links must be connected to the base through joints:

```xml
<!-- Problem: Unconnected link -->
<link name="floating_part"/>

<!-- Solution: Connect to base or another link -->
<joint name="connection" type="fixed">
  <parent link="base_link"/>
  <child link="floating_part"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### 2. Incorrect Axes

Ensure joint axes are properly oriented:

```xml
<!-- For a hinge that bends forward/backward -->
<axis xyz="1 0 0"/>  <!-- X-axis rotation -->

<!-- For a hinge that turns left/right -->
<axis xyz="0 1 0"/>  <!-- Y-axis rotation -->
```

### 3. Inertia Calculation Errors

Use proper formulas and realistic values:

```xml
<!-- Ensure all diagonal values are positive -->
<inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
```

## Practice Exercise

Create a simplified humanoid leg model with:
1. Hip joint (3 DOF approximation)
2. Knee joint (1 DOF revolute)
3. Ankle joint (2 DOF)
4. Proper mass and inertia properties
5. A force-torque sensor in the foot
6. Correct joint limits based on human anatomy

Validate your model using ROS tools and visualize it in RViz.

## Summary

Proper implementation of URDF links, joints, and sensors is fundamental to creating realistic humanoid robot models. Pay careful attention to physical properties, joint limits, and sensor placement to ensure your models are suitable for both simulation and eventual real-world deployment. The accuracy of these elements directly impacts the effectiveness of control algorithms and the realism of simulations.