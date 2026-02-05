---
sidebar_position: 3
title: "Connecting AI Logic to Robot Controllers"
---

# Connecting AI Logic to Robot Controllers

## Learning Objectives

After completing this section, you will be able to:
- Design architectures that connect AI algorithms to robot controllers
- Implement message passing between AI and control layers
- Create adaptive control systems that respond to AI decisions
- Apply state management techniques for AI-control interaction
- Evaluate performance and safety considerations for AI-control integration

## Introduction

The integration of Artificial Intelligence with robot controllers represents a critical aspect of modern robotics. This connection enables robots to make intelligent decisions based on sensory input and execute those decisions through precise physical control. Python, with its rich AI ecosystem and ROS 2's communication infrastructure, provides an excellent platform for implementing these AI-control interfaces.

## Architecture Patterns for AI-Control Integration

### 1. Direct Connection Pattern

In the simplest pattern, AI algorithms directly communicate with robot controllers:

```
AI Algorithm → ROS 2 Messages → Robot Controller
```

This pattern is suitable for:
- Simple reactive behaviors
- Direct mapping between AI decisions and control commands
- Low-latency requirements

Example implementation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class DirectAIControlNode(Node):
    def __init__(self):
        super().__init__('direct_ai_control')

        # Subscribe to sensor data
        self.sensor_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publish control commands
        self.control_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_control_loop)

        self.latest_scan = None

    def scan_callback(self, msg):
        self.latest_scan = np.array(msg.ranges)

    def ai_control_loop(self):
        if self.latest_scan is not None:
            # Simple AI decision: avoid obstacles
            control_cmd = self.obstacle_avoidance_ai(self.latest_scan)
            self.control_publisher.publish(control_cmd)

    def obstacle_avoidance_ai(self, scan_data):
        cmd = Twist()

        # Process scan data to detect obstacles
        front_range = scan_data[len(scan_data)//2 - 30 : len(scan_data)//2 + 30]
        min_front_dist = np.min(front_range[np.isfinite(front_range)])

        # Simple AI decision logic
        if min_front_dist > 1.0:
            cmd.linear.x = 0.5  # Move forward
        elif min_front_dist < 0.5:
            cmd.angular.z = 0.5  # Turn to avoid
        else:
            cmd.linear.x = 0.2  # Slow down

        return cmd
```

### 2. Mediator Pattern

A mediator node handles the communication between AI and control layers:

```
AI Algorithm → Mediator ← Robot Controller
```

This pattern is suitable for:
- Complex state management
- Coordination between multiple AI modules
- Advanced control strategies

Example implementation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class AICtrlMediatorNode(Node):
    def __init__(self):
        super().__init__('ai_ctrl_mediator')

        # AI communication
        self.ai_goal_sub = self.create_subscription(
            String,
            'ai_goals',
            self.ai_goal_callback,
            10
        )

        self.ai_status_pub = self.create_publisher(
            String,
            'ai_status',
            10
        )

        # Robot control communication
        self.ctrl_cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.robot_state_sub = self.create_subscription(
            Odometry,
            'odom',
            self.state_callback,
            10
        )

        # State management
        self.current_state = {'position': None, 'goal': None, 'status': 'idle'}
        self.ai_commands = []

    def ai_goal_callback(self, msg):
        # Receive goal from AI system
        self.current_state['goal'] = msg.data
        self.current_state['status'] = 'processing'

        # Process AI command and generate control commands
        self.process_ai_command(msg.data)

    def state_callback(self, msg):
        # Update robot state
        self.current_state['position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def process_ai_command(self, ai_command):
        # Convert AI decision to control commands
        if ai_command == 'move_to_target':
            cmd = Twist()
            cmd.linear.x = 0.5
            self.ctrl_cmd_pub.publish(cmd)
        elif ai_command == 'rotate_left':
            cmd = Twist()
            cmd.angular.z = 0.5
            self.ctrl_cmd_pub.publish(cmd)

        # Update status
        status_msg = String()
        status_msg.data = f'Executing: {ai_command}'
        self.ai_status_pub.publish(status_msg)
```

### 3. Hierarchical Pattern

Multiple layers of decision-making with different time scales:

```
High-Level AI → Mid-Level Planner → Low-Level Controller
```

This pattern is suitable for:
- Complex mission planning
- Multi-timescale control
- Resource management

## Implementing State Management

State management is crucial for maintaining consistency between AI decisions and robot behavior:

```python
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class RobotState(Enum):
    IDLE = 1
    NAVIGATING = 2
    MANIPULATING = 3
    EMERGENCY_STOP = 4

class StatefulAIControllerNode(Node):
    def __init__(self):
        super().__init__('stateful_ai_controller')

        self.state = RobotState.IDLE
        self.previous_state = None

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        # State transition timer
        self.state_timer = self.create_timer(0.05, self.state_machine)

        # State data
        self.joint_positions = {}
        self.target_reached = False

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]

    def state_machine(self):
        # State transition logic
        if self.state != self.previous_state:
            self.handle_state_transition()

        # Execute current state behavior
        if self.state == RobotState.NAVIGATING:
            self.execute_navigation()
        elif self.state == RobotState.MANIPULATING:
            self.execute_manipulation()
        elif self.state == RobotState.EMERGENCY_STOP:
            self.execute_emergency_stop()

        self.previous_state = self.state

    def handle_state_transition(self):
        if self.state == RobotState.NAVIGATING:
            self.get_logger().info('Transitioning to navigation mode')
        elif self.state == RobotState.MANIPULATING:
            self.get_logger().info('Transitioning to manipulation mode')
        elif self.state == RobotState.EMERGENCY_STOP:
            self.get_logger().info('Emergency stop activated')

    def execute_navigation(self):
        # Navigation-specific control logic
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.1
        self.cmd_pub.publish(cmd)

    def execute_manipulation(self):
        # Manipulation-specific control logic
        pass

    def execute_emergency_stop(self):
        # Emergency stop control logic
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def update_state_from_ai(self, ai_decision):
        # Update state based on AI decision
        if ai_decision == 'navigate':
            if self.state != RobotState.NAVIGATING:
                self.state = RobotState.NAVIGATING
        elif ai_decision == 'manipulate':
            if self.state != RobotState.MANIPULATING:
                self.state = RobotState.MANIPULATING
        elif ai_decision == 'emergency_stop':
            self.state = RobotState.EMERGENCY_STOP
```

## Integrating Machine Learning Models

Python's ML ecosystem makes it easy to integrate trained models:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class MLControlNode(Node):
    def __init__(self):
        super().__init__('ml_control_node')

        # Image processing
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Load pre-trained model (example with dummy model)
        # self.model = self.load_trained_model()

        self.model_loaded = True  # Placeholder for model loading

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.model_loaded:
                # Process image with ML model
                control_command = self.infer_control_from_image(cv_image)

                # Publish control command
                self.cmd_pub.publish(control_command)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def infer_control_from_image(self, image):
        # Preprocess image
        processed_img = self.preprocess_image(image)

        # Run inference (dummy implementation)
        # prediction = self.model.predict(processed_img)

        # For demo purposes, use simple computer vision
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        moments = cv2.moments(mask)

        cmd = Twist()
        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            image_center = image.shape[1] / 2

            # Simple proportional control to center of red object
            error = cx - image_center
            cmd.angular.z = -error * 0.005  # Proportional gain
            cmd.linear.x = 0.3
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0

        return cmd

    def preprocess_image(self, image):
        # Resize and normalize image for model input
        resized = cv2.resize(image, (224, 224))
        normalized = resized.astype(np.float32) / 255.0
        return np.expand_dims(normalized, axis=0)
```

## Safety Considerations

When connecting AI to robot controllers, safety is paramount:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SafeAIControllerNode(Node):
    def __init__(self):
        super().__init__('safe_ai_controller')

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)

        # Safety parameters
        self.safety_threshold = 0.5  # meters
        self.emergency_stop_active = False
        self.last_safe_command = Twist()

        # Rate limiting
        self.command_timer = self.create_timer(0.1, self.process_commands)

        self.latest_scan = None
        self.pending_command = None

    def scan_callback(self, msg):
        self.latest_scan = np.array(msg.ranges)

    def set_command(self, cmd):
        # Validate command before accepting
        if self.validate_command(cmd):
            self.pending_command = cmd
        else:
            self.get_logger().warn('Invalid command rejected by safety validator')

    def validate_command(self, cmd):
        # Check if command is within safe limits
        if abs(cmd.linear.x) > 2.0:  # Max linear speed 2 m/s
            return False
        if abs(cmd.angular.z) > 1.5:  # Max angular speed 1.5 rad/s
            return False
        return True

    def process_commands(self):
        if self.emergency_stop_active:
            # Always send stop command when emergency stop is active
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            return

        if self.pending_command is not None and self.latest_scan is not None:
            # Check safety before executing command
            if self.is_path_clear(self.pending_command):
                self.cmd_pub.publish(self.pending_command)
                self.last_safe_command = self.pending_command
            else:
                # Path is not clear, execute emergency stop
                self.emergency_stop()
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

    def is_path_clear(self, cmd):
        if self.latest_scan is None:
            return False

        # Check if planned movement is safe based on current scan
        if cmd.linear.x > 0:  # Moving forward
            front_scan = self.latest_scan[len(self.latest_scan)//2 - 30 : len(self.latest_scan)//2 + 30]
            min_front_dist = np.min(front_scan[np.isfinite(front_scan)])
            return min_front_dist > self.safety_threshold
        elif cmd.linear.x < 0:  # Moving backward
            back_scan = np.concatenate([
                self.latest_scan[:30],  # Left rear
                self.latest_scan[-30:]  # Right rear
            ])
            min_back_dist = np.min(back_scan[np.isfinite(back_scan)])
            return min_back_dist > self.safety_threshold

        return True  # No linear movement, check angular safety separately

    def emergency_stop(self):
        self.emergency_stop_active = True
        self.get_logger().error('EMERGENCY STOP ACTIVATED')

        # Reset after a delay
        self.create_timer(2.0, self.reset_emergency_stop)

    def reset_emergency_stop(self):
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop reset')
```

## Performance Optimization

For efficient AI-control integration:

```python
import rclpy
from rclpy.node import Node
from threading import Lock
import time

class OptimizedAIControllerNode(Node):
    def __init__(self):
        super().__init__('optimized_ai_controller')

        # Use threading for computationally expensive AI operations
        self.data_lock = Lock()
        self.ai_result = None
        self.last_update_time = time.time()

        # Optimize for frequency
        self.ai_timer = self.create_timer(0.05, self.run_ai_computation)  # 20 Hz
        self.control_timer = self.create_timer(0.01, self.send_control_commands)  # 100 Hz

    def run_ai_computation(self):
        # Run AI computations at lower frequency
        with self.data_lock:
            # Perform AI computation here
            self.ai_result = self.expensive_ai_computation()

    def send_control_commands(self):
        # Send control commands at higher frequency
        with self.data_lock:
            if self.ai_result is not None:
                # Apply smoothing/filtering to control commands
                smoothed_cmd = self.smooth_command(self.ai_result)
                # Publish command
                pass

    def expensive_ai_computation(self):
        # Placeholder for actual AI computation
        return "result"

    def smooth_command(self, raw_cmd):
        # Apply smoothing to reduce jerky movements
        return raw_cmd
```

## Practice Exercise

Design and implement a Python node that connects a simple path-planning AI to a differential drive robot controller. The system should:

1. Take a goal position as input
2. Plan a path to the goal
3. Generate velocity commands to follow the path
4. Include safety checks to avoid obstacles
5. Provide feedback to the AI system about execution status

Test your implementation in simulation with various goal positions and obstacle configurations.

## Summary

Connecting AI logic to robot controllers requires careful consideration of architecture, state management, safety, and performance. By implementing proper communication patterns, state management, and safety checks, you can create robust systems that effectively combine artificial intelligence with precise physical control. The flexibility of Python and the communication infrastructure of ROS 2 make this integration both powerful and accessible.