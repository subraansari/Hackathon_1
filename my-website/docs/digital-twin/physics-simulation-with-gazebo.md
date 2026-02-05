---
sidebar_position: 1
title: "Physics Simulation with Gazebo"
---

# Physics Simulation with Gazebo

## Learning Objectives

After completing this chapter, you will be able to:
- Set up and configure Gazebo for humanoid robot simulation
- Understand and configure gravity, collision detection, and joint dynamics
- Create realistic robot-world interactions
- Tune physics parameters for accurate simulation

## Introduction

Gazebo is a powerful physics simulator that provides realistic simulation of robots in complex environments. For humanoid robots, accurate physics simulation is crucial for developing and testing control algorithms before deployment on real hardware. This chapter will guide you through the fundamentals of physics simulation with Gazebo, focusing on the elements most important for humanoid robotics.

## Gazebo Architecture and Components

### Physics Engine

Gazebo uses the Open Dynamics Engine (ODE), Bullet, or SimBody physics engines to simulate realistic physical interactions. Each engine has its strengths:

- **ODE**: Good for most applications, well-tested
- **Bullet**: Better for complex contacts and soft body dynamics
- **SimBody**: High-fidelity simulation for biomechanics

### World Files

World files define the environment, lighting, physics parameters, and initial conditions:

```xml
<sdf version="1.7">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Gravity Configuration

Gravity is fundamental to realistic humanoid simulation. The default gravity is set to Earth's gravity:

```xml
<world>
  <gravity>0 0 -9.8</gravity>
</world>
```

For humanoid robots, you might need to adjust gravity for different scenarios:
- Lower gravity for moon/mars simulation
- Zero gravity for space robotics
- Different orientations for climbing robots

### Custom Gravity Example:

```xml
<!-- Moon gravity (1/6 of Earth) -->
<gravity>0 0 -1.63</gravity>

<!-- Zero gravity -->
<gravity>0 0 0</gravity>
```

## Collision Detection

Collision detection is critical for humanoid robots to interact realistically with their environment:

### Collision Properties

Each link in your robot model needs collision properties:

```xml
<link name="upper_arm">
  <collision name="collision">
    <geometry>
      <capsule>
        <radius>0.05</radius>
        <length>0.3</length>
      </capsule>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.01</restitution_coefficient>
        <threshold>100000</threshold>
      </bounce>
      <contact>
        <ode>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1e+13</kp>
          <kd>1</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

### Collision Shapes

Different shapes provide different performance and accuracy characteristics:

- **Box**: Simple, fast, good for torso and limbs
- **Cylinder**: Good for limbs and cylindrical objects
- **Capsule**: Best for limbs, handles contact naturally
- **Sphere**: Simple, good for balls and rounded objects
- **Mesh**: Most accurate but computationally expensive

## Joint Dynamics

Joints connect robot links and must be properly configured for realistic simulation:

### Joint Types for Humanoid Robots

```xml
<!-- Revolute joint for elbow/knee -->
<joint name="elbow_joint" type="revolute">
  <parent>upper_arm</parent>
  <child>lower_arm</child>
  <axis>
    <xyz>1 0 0</xyz>  <!-- Rotation around X-axis -->
    <limit>
      <lower>-2.356</lower>  <!-- -135 degrees -->
      <upper>0</upper>       <!-- 0 degrees -->
      <effort>100</effort>
      <velocity>2</velocity>
    </limit>
    <dynamics>
      <damping>0.5</damping>
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>

<!-- Continuous joint for shoulder -->
<joint name="shoulder_yaw" type="continuous">
  <parent>torso</parent>
  <child>upper_arm</child>
  <axis>
    <xyz>0 1 0</xyz>  <!-- Rotation around Y-axis -->
    <dynamics>
      <damping>0.3</damping>
      <friction>0.05</friction>
    </dynamics>
  </axis>
</joint>
```

### Joint Limit Considerations

For humanoid robots, joint limits should reflect human anatomical constraints:

- **Shoulder**: Complex, multi-DOF (pitch: -90° to 90°, yaw: -90° to 90°, roll: -120° to 120°)
- **Elbow**: -150° to 0° flexion
- **Hip**: Flexion -30° to 70°, abduction -20° to 20°, rotation -30° to 30°
- **Knee**: 0° to 150° flexion
- **Ankle**: Dorsiflexion -30° to 30°, inversion/eversion -20° to 20°

## Robot-World Interaction

### Contact Sensors

To detect when your robot makes contact with the environment:

```xml
<sensor name="contact_sensor" type="contact">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <contact>
    <collision>foot_collision</collision>
  </contact>
</sensor>
```

### Ground Reaction Forces

For humanoid robots, understanding ground reaction forces is crucial for balance:

```xml
<gazebo reference="left_foot">
  <sensor name="left_foot_force" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

## Physics Parameter Tuning

### Time Step Configuration

For stable humanoid simulation:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
  <real_time_update_rate>1000.0</real_time_update_rate>  <!-- 1000 Hz -->
  <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
</physics>
```

Smaller time steps provide more accurate simulation but require more computational power.

### Solver Parameters

Fine-tune the physics solver for your specific robot:

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>  <!-- or "world" for more accuracy -->
      <iters>10</iters>   <!-- Iterations per step -->
      <sor>1.3</sor>     <!-- Successive over-relaxation parameter -->
    </solver>
    <constraints>
      <cfm>0</cfm>        <!-- Constraint force mixing -->
      <erp>0.2</erp>     <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Best Practices for Humanoid Simulation

### 1. Mass Distribution

Ensure realistic mass distribution in your URDF:

```xml
<inertial>
  <mass value="1.5"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- Use proper inertia values based on geometry -->
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
</inertial>
```

### 2. Collision vs Visual Separation

Use simple collision geometries for performance:

```xml
<link name="complex_visual_link">
  <visual>
    <geometry>
      <mesh filename="complex_shape.dae"/>  <!-- Detailed visual -->
    </geometry>
  </visual>
  <collision>
    <geometry>
      <capsule radius="0.05" length="0.3"/>  <!-- Simple collision -->
    </geometry>
  </collision>
</link>
```

### 3. Stability Considerations

For stable humanoid simulation:
- Use appropriate damping values
- Set realistic friction coefficients
- Ensure proper mass distribution
- Use sufficient simulation frequency

## Troubleshooting Common Issues

### Instability

If your robot is unstable in simulation:
- Check mass and inertia values
- Increase damping in joint dynamics
- Reduce time step size
- Verify joint limits are realistic

### Penetration

If links are penetrating each other or the environment:
- Increase ERP (Error Reduction Parameter)
- Decrease CFM (Constraint Force Mixing)
- Use stiffer contact parameters (higher kp)
- Check collision geometry overlap

### Performance Issues

If simulation is slow:
- Simplify collision geometries
- Increase time step (within stability limits)
- Reduce solver iterations
- Limit number of contacts

## Practice Exercise

Create a simple humanoid leg model with proper physics configuration:
1. Define realistic joint limits for hip, knee, and ankle
2. Configure appropriate mass and inertia properties
3. Set up collision detection for the foot
4. Add contact sensors to detect ground interaction

Test your model in Gazebo and observe how it interacts with the ground plane.

## Summary

Physics simulation with Gazebo provides the foundation for realistic humanoid robot development. By properly configuring gravity, collisions, and joint dynamics, you can create simulations that closely match real-world behavior. This enables safer, faster, and more cost-effective development of humanoid robots before deployment on physical hardware.