---
sidebar_position: 5
title: "Navigation with Nav2"
---

# Navigation with Nav2

## Learning Objectives

After completing this chapter, you will be able to:
- Configure Navigation2 for humanoid robot navigation
- Integrate Isaac Sim with Nav2 for simulation-based development
- Implement path planning algorithms for humanoid kinematics
- Set up navigation parameters for complex humanoid robots
- Validate navigation performance in Isaac Sim environments
- Troubleshoot common navigation issues with humanoid robots

## Introduction

Navigation2 (Nav2) is the ROS 2 navigation stack that provides a complete navigation system for mobile robots. When combined with Isaac Sim, it enables sophisticated navigation development and testing for humanoid robots. This chapter focuses on configuring and utilizing Nav2 specifically for humanoid robot navigation, taking into account the unique kinematic constraints and requirements of humanoid platforms.

Nav2 provides a comprehensive navigation system that includes:
- Global path planning for route finding
- Local path planning for obstacle avoidance
- Controller algorithms for robot motion
- Recovery behaviors for challenging situations
- Behavior trees for complex navigation tasks

## Nav2 Architecture Overview

### Core Components

Nav2 consists of several key components that work together to provide navigation capabilities:

#### 1. Global Planner
- Computes optimal paths from start to goal
- Considers static map information
- Accounts for robot footprint and kinematics
- Supports multiple planning algorithms (NavFn, GlobalPlanner, etc.)

#### 2. Local Planner
- Handles real-time obstacle avoidance
- Maintains the robot's position along the global path
- Executes dynamic obstacle avoidance
- Manages robot motion in local environment

#### 3. Controller
- Translates navigation goals into robot motion commands
- Manages robot velocity and acceleration
- Ensures smooth and safe robot movement
- Handles kinematic constraints

#### 4. Behavior Trees
- Orchestrates navigation tasks
- Manages recovery behaviors
- Handles complex navigation scenarios
- Provides modular navigation logic

### Integration with Isaac Sim

The integration between Isaac Sim and Nav2 enables:
- Photorealistic simulation of navigation scenarios
- Accurate sensor simulation for navigation
- Physics-based validation of navigation performance
- Synthetic data generation for navigation training

## Configuring Nav2 for Humanoid Robots

### Understanding Humanoid Kinematics

Humanoid robots have unique kinematic constraints that must be considered in navigation:

#### 1. Footprint Considerations
- Larger robot footprint than wheeled robots
- Complex collision geometry
- Dynamic stability requirements
- Balance constraints during movement

#### 2. Motion Constraints
- Bipedal locomotion patterns
- Limited turning radius
- Step height and length constraints
- Balance and stability requirements

### Nav2 Configuration for Humanoid Robots

```yaml
# config/nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitor: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the custom behavior tree for humanoid navigation
    # Use a tree that accounts for humanoid-specific constraints
    default_nav_through_poses_bt_xml: "humanoid_nav_through_poses.xml"
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 32
      control_freq: 30.0
      horizon: 3.2
      Q: [3.0, 0.0, 0.1, 0.0, 0.05, 0.0, 0.1]
      R: [1.0, 1.0, 1.0]
      P: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      controller_frequency: 20.0
      # Humanoid-specific parameters
      max_linear_speed: 0.5  # Slower for stability
      min_linear_speed: 0.05
      max_angular_speed: 0.3
      min_angular_speed: 0.1
      # Footstep planning considerations
      step_size: 0.3  # Typical humanoid step size

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.4  # Larger for humanoid
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.4  # Larger for humanoid
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.85

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size: 0.025  # Finer resolution for humanoid
      min_distance_from_robot: 0.2  # Clearance for humanoid feet

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### Humanoid-Specific Parameters

#### 1. Robot Radius
```yaml
# For humanoid robots, the radius needs to account for the larger footprint
robot_radius: 0.4  # Increased from typical wheeled robot
```

#### 2. Velocity Limits
```yaml
# Humanoid robots typically have slower speeds for stability
max_linear_speed: 0.5    # Slower for stability
min_linear_speed: 0.05   # Minimum to maintain momentum
max_angular_speed: 0.3   # Slower turning for balance
```

#### 3. Footstep Planning Considerations
```yaml
# Step size parameters for humanoid locomotion
step_size: 0.3  # Typical humanoid step size
```

## Isaac Sim Integration

### Isaac Sim Navigation Setup

Setting up Isaac Sim to work with Nav2 requires specific configuration:

#### 1. Robot Model Configuration
- Proper URDF with accurate collision and visual geometries
- Correct joint limits reflecting humanoid capabilities
- Appropriate mass and inertia properties

#### 2. Sensor Configuration
- LiDAR simulation for navigation
- Camera sensors for perception
- IMU for balance and localization

### Example: Isaac Sim Nav2 Launch

```python
# launch/isaac_nav2.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Navigation launch file
    nav2_launch_file_dir = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])

    # RViz configuration
    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('isaac_nav2_examples'),
        'rviz',
        'isaac_nav2_config.rviz'
    ])

    # Include Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file_dir),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('isaac_nav2_examples'),
                'config',
                'nav2_params_humanoid.yaml'
            ])
        }.items()
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup,
        rviz_node
    ])
```

### Isaac Sim Robot Controller Integration

For humanoid robots, the controller needs to interface with Isaac Sim's physics engine:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class IsaacHumanoidController(Node):
    def __init__(self):
        super().__init__('isaac_humanoid_controller')

        # Subscribe to navigation velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish joint commands for Isaac Sim
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/isaac_joint_commands',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Store velocity commands
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Humanoid-specific parameters
        self.step_size = 0.3  # Typical humanoid step
        self.step_height = 0.05  # Lift foot slightly
        self.balance_margin = 0.1  # Safety margin for balance

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def control_loop(self):
        # Convert velocity commands to humanoid-specific joint movements
        joint_commands = self.calculate_humanoid_motion(
            self.linear_x, self.angular_z
        )

        # Publish joint commands to Isaac Sim
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = joint_commands['names']
        joint_msg.position = joint_commands['positions']
        joint_msg.velocity = joint_commands['velocities']
        joint_msg.effort = joint_commands['efforts']

        self.joint_cmd_pub.publish(joint_msg)

    def calculate_humanoid_motion(self, linear_x, angular_z):
        # This is a simplified example
        # Real implementation would use complex humanoid locomotion patterns
        # such as inverse kinematics, footstep planning, and balance control

        joint_names = []
        joint_positions = []
        joint_velocities = []
        joint_efforts = []

        # Example: Simple walking gait pattern
        current_time = self.get_clock().now().nanoseconds / 1e9
        phase = (current_time * 2 * math.pi * 0.5) % (2 * math.pi)  # 0.5 Hz walking

        # Calculate joint positions based on walking phase and desired velocity
        # This would be much more complex in a real humanoid implementation
        for i, joint_name in enumerate(['left_hip', 'right_hip', 'left_knee', 'right_knee']):
            # Calculate position based on phase and desired motion
            pos = self.calculate_joint_position(joint_name, phase, linear_x, angular_z)

            joint_names.append(joint_name)
            joint_positions.append(pos)
            joint_velocities.append(0.0)  # Simplified
            joint_efforts.append(0.0)    # Simplified

        return {
            'names': joint_names,
            'positions': joint_positions,
            'velocities': joint_velocities,
            'efforts': joint_efforts
        }

    def calculate_joint_position(self, joint_name, phase, linear_x, angular_z):
        # Simplified joint position calculation
        # In reality, this would use inverse kinematics and gait planning
        base_pos = 0.0

        if 'hip' in joint_name:
            # Hip movement for walking
            if joint_name.startswith('left'):
                base_pos = math.sin(phase) * linear_x * 0.1
            else:
                base_pos = math.sin(phase + math.pi) * linear_x * 0.1
        elif 'knee' in joint_name:
            # Knee movement synchronized with hip
            if joint_name.startswith('left'):
                base_pos = math.cos(phase) * linear_x * 0.05
            else:
                base_pos = math.cos(phase + math.pi) * linear_x * 0.05

        return base_pos
```

## Path Planning for Humanoid Robots

### Humanoid Kinematic Constraints

Humanoid robots have specific kinematic constraints that affect path planning:

#### 1. Step Height Limitations
- Maximum step height for obstacle traversal
- Need for ramp or stair navigation planning
- Consideration of ground clearance

#### 2. Turning Radius
- Limited ability to turn in place
- Need for wider turning maneuvers
- Balance considerations during turns

#### 3. Stability Requirements
- Center of mass management
- Support polygon maintenance
- Dynamic balance during movement

### Custom Path Planning Algorithms

For humanoid robots, custom path planning may be necessary:

```python
import numpy as np
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path

class HumanoidPathPlanner:
    def __init__(self):
        # Humanoid-specific parameters
        self.step_height_limit = 0.15  # Maximum step height
        self.step_length = 0.3        # Typical step length
        self.turn_radius = 0.5        # Minimum turning radius
        self.support_polygon_radius = 0.2  # Balance support radius

    def plan_path_with_humanoid_constraints(self, start_pose, goal_pose, costmap):
        # First, run standard path planning
        base_path = self.run_standard_planner(start_pose, goal_pose, costmap)

        # Then, adapt the path for humanoid constraints
        humanoid_path = self.adapt_path_for_humanoid(base_path, costmap)

        return humanoid_path

    def adapt_path_for_humanoid(self, base_path, costmap):
        # Adapt path waypoints for humanoid kinematics
        adapted_path = Path()
        adapted_path.header = base_path.header

        for i, pose in enumerate(base_path.poses):
            # Check if this waypoint is feasible for humanoid
            if self.is_waypoint_feasible(pose, costmap):
                # Add the waypoint
                adapted_path.poses.append(pose)
            else:
                # Find alternative route that's feasible
                feasible_pose = self.find_feasible_alternative(pose, costmap)
                if feasible_pose:
                    adapted_path.poses.append(feasible_pose)

        # Smooth the path for humanoid gait
        smoothed_path = self.smooth_humanoid_path(adapted_path)
        return smoothed_path

    def is_waypoint_feasible(self, pose, costmap):
        # Check if waypoint is feasible considering:
        # - Obstacle clearance for humanoid size
        # - Step height limitations
        # - Slope limitations for bipedal walking
        # - Stability constraints

        # Check robot footprint at this pose
        robot_radius = 0.4  # Humanoid radius
        if self.has_obstacles_in_radius(pose, robot_radius, costmap):
            return False

        # Check if terrain is traversable (not too steep)
        if self.terrain_slope_too_steep(pose, costmap):
            return False

        # Check step height constraints
        if self.step_height_exceeds_limit(pose, costmap):
            return False

        return True

    def smooth_humanoid_path(self, path):
        # Apply smoothing that respects humanoid kinematics
        # This might include creating waypoints that follow
        # natural walking patterns and maintain balance

        smoothed_path = Path()
        smoothed_path.header = path.header

        if len(path.poses) < 2:
            return path

        # Use a smoothing algorithm that considers humanoid gait
        for i in range(len(path.poses)):
            if i == 0 or i == len(path.poses) - 1:
                # Keep start and end points
                smoothed_path.poses.append(path.poses[i])
            else:
                # Smooth intermediate points with humanoid-aware algorithm
                smoothed_pose = self.humanoid_smooth_point(path.poses, i)
                smoothed_path.poses.append(smoothed_pose)

        return smoothed_path

    def humanoid_smooth_point(self, poses, idx):
        # Humanoid-aware smoothing that maintains stability
        # This would implement more sophisticated smoothing
        # considering balance and natural walking patterns
        current_pose = poses[idx]

        # For now, return the original pose
        # In practice, this would apply humanoid-aware smoothing
        return current_pose

    def has_obstacles_in_radius(self, pose, radius, costmap):
        # Check for obstacles within the specified radius
        # Implementation would check costmap values
        return False  # Simplified for example

    def terrain_slope_too_steep(self, pose, costmap):
        # Check if terrain slope exceeds humanoid capability
        return False  # Simplified for example

    def step_height_exceeds_limit(self, pose, costmap):
        # Check if step height exceeds humanoid capability
        return False  # Simplified for example

    def find_feasible_alternative(self, original_pose, costmap):
        # Find an alternative waypoint that's feasible
        return original_pose  # Simplified for example

    def run_standard_planner(self, start_pose, goal_pose, costmap):
        # Run standard Nav2 path planner
        # This would interface with Nav2's planning plugins
        path = Path()
        path.header.frame_id = "map"

        # Add start and goal as simple path
        path.poses.append(start_pose)
        path.poses.append(goal_pose)

        return path
```

## Navigation Behavior Trees for Humanoid Robots

### Custom Behavior Trees

Humanoid robots may require custom behavior trees that account for their unique capabilities:

```xml
<!-- humanoid_nav_to_pose.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="NavigateToPose">
            <GoalUpdated/>
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <FollowPath path="{path}" controller_id="FollowPath"/>
        </Sequence>
    </BehaviorTree>
</root>

<!-- Enhanced version for humanoid-specific behaviors -->
<!-- humanoid_nav_to_pose_enhanced.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="NavigateToPose">
            <GoalUpdated/>
            <!-- Check if goal is humanoid-feasible -->
            <IsHumanoidGoalFeasible goal="{goal}" result="{goal_feasible}"/>
            <Fallback name="GoalProcessing">
                <Sequence name="ProcessFeasibleGoal">
                    <Condition input="{goal_feasible}"/>
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                    <Fallback name="NavigationWithRecovery">
                        <Sequence name="Navigate">
                            <HumanoidFollowPath path="{path}" controller_id="FollowPath"/>
                            <Succeeded/>
                        </Sequence>
                        <ReactiveFallback name="RecoveryFallback">
                            <RecoveryNode recovery_behavior_id="spin"/>
                            <RecoveryNode recovery_behavior_id="backup"/>
                            <RecoveryNode recovery_behavior_id="humanoid_wait_balance"/>
                        </ReactiveFallback>
                    </Fallback>
                </Sequence>
                <HumanoidHandleInfeasibleGoal goal="{goal}"/>
            </Fallback>
        </Sequence>
    </BehaviorTree>
</root>
```

### Humanoid-Specific Recovery Behaviors

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import Recover
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class HumanoidRecoveryNode(Node):
    def __init__(self):
        super().__init__('humanoid_recovery_node')

        # Action server for recovery behaviors
        self.recover_action_server = ActionServer(
            self,
            Recover,
            'recover',
            self.execute_recovery_callback
        )

        # Velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Balance monitor
        self.balance_sub = self.create_subscription(
            Bool,
            '/balance_ok',
            self.balance_callback,
            10
        )

        self.balance_ok = True

    def balance_callback(self, msg):
        self.balance_ok = msg.data

    def execute_recovery_callback(self, goal_handle):
        recovery_type = goal_handle.request.behavior

        if recovery_type == 'humanoid_wait_balance':
            result = self.execute_wait_balance_recovery()
        elif recovery_type == 'humanoid_step_adjust':
            result = self.execute_step_adjust_recovery()
        else:
            # Fall back to standard recovery
            result = self.execute_standard_recovery(recovery_type)

        goal_handle.succeed()
        return result

    def execute_wait_balance_recovery(self):
        # Wait until humanoid regains balance
        self.get_logger().info('Executing humanoid balance recovery')

        # Stop robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        # Wait for balance to be restored
        timeout = self.get_clock().now() + Duration(seconds=5.0)
        while not self.balance_ok and self.get_clock().now() < timeout:
            self.get_logger().info('Waiting for balance to be restored...')
            time.sleep(0.1)

        result = Recover.Result()
        result.completed = self.balance_ok
        return result

    def execute_step_adjust_recovery(self):
        # Adjust footsteps to regain stable position
        self.get_logger().info('Executing humanoid step adjustment recovery')

        # Implementation would involve adjusting planned footsteps
        # to move robot to a more stable position
        result = Recover.Result()
        result.completed = True
        return result

    def execute_standard_recovery(self, recovery_type):
        # Execute standard Nav2 recovery behavior
        result = Recover.Result()
        result.completed = True
        return result
```

## Validation and Testing in Isaac Sim

### Isaac Sim Navigation Testing

Testing navigation in Isaac Sim involves several validation steps:

#### 1. Simulation Setup
- Create realistic environments with obstacles
- Configure proper physics parameters
- Set up appropriate sensor models

#### 2. Performance Metrics
- Path optimality
- Navigation success rate
- Time to goal
- Collision avoidance effectiveness
- Stability during navigation

### Example: Navigation Validation Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
import numpy as np

class NavigationValidator(Node):
    def __init__(self):
        super().__init__('navigation_validator')

        # Subscribe to robot pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscribe to goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )

        # Subscribe to commanded velocity
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for validation metrics
        self.path_efficiency_pub = self.create_publisher(
            Float64,
            '/validation/path_efficiency',
            10
        )
        self.success_rate_pub = self.create_publisher(
            Float64,
            '/validation/success_rate',
            10
        )

        # Storage for validation data
        self.robot_path = []
        self.current_goal = None
        self.start_pose = None
        self.navigation_start_time = None
        self.navigation_success = False

    def odom_callback(self, msg):
        # Store robot path for analysis
        current_pose = msg.pose.pose
        self.robot_path.append(current_pose)

        if self.start_pose is None:
            self.start_pose = current_pose
            self.navigation_start_time = self.get_clock().now()

        # Calculate current distance to goal
        if self.current_goal:
            dist_to_goal = self.calculate_distance(current_pose, self.current_goal.pose)

            if dist_to_goal < 0.5:  # Goal reached
                self.navigation_success = True
                self.publish_validation_metrics()

    def goal_callback(self, msg):
        self.current_goal = msg
        self.reset_navigation_metrics()

    def cmd_vel_callback(self, msg):
        # Monitor commanded velocities for analysis
        linear_speed = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
        angular_speed = abs(msg.angular.z)

        # Check for oscillation or stuck behavior
        if linear_speed < 0.01 and angular_speed < 0.01:
            self.get_logger().warn('Robot appears to be stuck')

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)

    def publish_validation_metrics(self):
        if len(self.robot_path) < 2 or not self.start_pose or not self.current_goal:
            return

        # Calculate path efficiency (actual path length vs straight-line distance)
        actual_path_length = self.calculate_path_length(self.robot_path)
        straight_line_distance = self.calculate_distance(
            self.start_pose,
            self.current_goal.pose
        )

        if straight_line_distance > 0:
            path_efficiency = actual_path_length / straight_line_distance
        else:
            path_efficiency = 1.0  # Already at goal

        # Publish metrics
        efficiency_msg = Float64()
        efficiency_msg.data = path_efficiency
        self.path_efficiency_pub.publish(efficiency_msg)

        success_msg = Float64()
        success_msg.data = 1.0 if self.navigation_success else 0.0
        self.success_rate_pub.publish(success_msg)

    def calculate_path_length(self, path):
        total_length = 0.0
        for i in range(1, len(path)):
            segment_length = self.calculate_distance(path[i-1], path[i])
            total_length += segment_length
        return total_length

    def reset_navigation_metrics(self):
        self.robot_path = []
        self.start_pose = None
        self.navigation_start_time = None
        self.navigation_success = False

def main():
    rclpy.init()
    validator = NavigationValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()
```

## Troubleshooting Navigation Issues

### Common Navigation Problems with Humanoid Robots

#### 1. Instability During Navigation
- Symptoms: Robot falls over during movement
- Causes: Improper balance control, aggressive velocity commands
- Solutions: Reduce speeds, improve balance algorithms, adjust PID gains

#### 2. Path Following Inaccuracies
- Symptoms: Robot deviates from planned path
- Causes: Kinematic constraints, sensor inaccuracies
- Solutions: Account for humanoid kinematics in planning, improve localization

#### 3. Collision Avoidance Failures
- Symptoms: Robot collides with obstacles
- Causes: Large robot footprint, delayed reactions
- Solutions: Increase safety margins, improve sensor coverage, faster reactions

### Debugging Tools

#### 1. RViz Visualization
- Use Nav2's built-in visualization tools
- Monitor costmaps, global/local paths, and robot pose
- Visualize sensor data and obstacle detection

#### 2. Parameter Tuning
- Use runtime parameter reconfiguration
- Test different values for key parameters
- Monitor performance metrics during testing

## Practice Exercise

Create a complete navigation system for a humanoid robot in Isaac Sim that includes:

1. Configure Nav2 parameters specifically for humanoid kinematics
2. Implement a custom path planner that accounts for humanoid constraints
3. Create behavior trees for humanoid-specific navigation tasks
4. Test navigation in various Isaac Sim environments
5. Validate navigation performance using appropriate metrics

Evaluate the system's performance in terms of path efficiency, success rate, and stability during navigation.

## Summary

Navigation with Nav2 for humanoid robots requires careful consideration of the unique kinematic constraints and capabilities of bipedal robots. By properly configuring Nav2 parameters, accounting for humanoid-specific constraints in path planning, and integrating with Isaac Sim for testing and validation, you can create robust navigation systems for humanoid robots. The combination of Nav2's mature navigation stack with Isaac Sim's realistic simulation capabilities provides an excellent platform for developing and validating humanoid navigation systems before deployment on real hardware.