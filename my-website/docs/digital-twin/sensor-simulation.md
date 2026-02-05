---
sidebar_position: 2
title: "Sensor Simulation"
---

# Sensor Simulation

## Learning Objectives

After completing this chapter, you will be able to:
- Simulate various types of sensors including LiDAR, depth cameras, and IMUs
- Configure sensor parameters for realistic simulation
- Process simulated sensor data in ROS 2
- Create sensor data pipelines for robot perception
- Validate sensor simulation accuracy

## Introduction

Sensor simulation is a critical component of digital twin technology for robotics. Realistic sensor simulation allows you to test perception algorithms, localization, and navigation systems in a controlled environment before deploying to real hardware. This chapter covers the simulation of common sensors used in humanoid robotics, including LiDAR, depth cameras, and IMUs, and how to integrate them into your Gazebo simulation.

## LiDAR Simulation

LiDAR (Light Detection and Ranging) sensors are essential for navigation, mapping, and obstacle detection in robotics. Gazebo provides realistic LiDAR simulation with configurable parameters.

### Basic LiDAR Configuration

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>  <!-- Number of rays per revolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -π radians (-180 degrees) -->
          <max_angle>3.14159</max_angle>   <!-- π radians (180 degrees) -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>     <!-- Minimum range (meters) -->
        <max>30.0</max>    <!-- Maximum range (meters) -->
        <resolution>0.01</resolution>  <!-- Range resolution (meters) -->
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_frame</frame_name>
    </plugin>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Advanced LiDAR Configuration

For more sophisticated LiDAR simulation, you can configure multi-layer sensors:

```xml
<sensor name="3d_lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
        <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>120.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="velodyne_controller" filename="libgazebo_ros_velodyne_gpu_laser.so">
    <ros>
      <namespace>/velodyne</namespace>
      <remapping>~/out:=points</remapping>
    </ros>
    <topic_name>points</topic_name>
    <frame_name>velodyne_frame</frame_name>
    <min_range>0.9</min_range>
    <max_range>130.0</max_range>
    <gaussian_noise>0.008</gaussian_noise>
  </plugin>
</sensor>
```

### LiDAR Noise and Accuracy

Real LiDAR sensors have noise and accuracy limitations. Simulate these characteristics:

```xml
<ray>
  <range>
    <min>0.1</min>
    <max>30.0</max>
    <resolution>0.01</resolution>
  </range>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm accuracy -->
  </noise>
</ray>
```

## Depth Camera Simulation

Depth cameras provide 3D spatial information crucial for humanoid robot perception, manipulation, and navigation.

### Basic Depth Camera Configuration

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>image_raw:=rgb/image_raw</remapping>
        <remapping>depth/image_raw:=depth/image_raw</remapping>
        <remapping>depth/camera_info:=depth/camera_info</remapping>
      </ros>
      <frame_name>camera_depth_optical_frame</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

### RGB-D Camera Pipeline

Configure the RGB-D camera to publish multiple data streams:

```xml
<sensor name="rgbd_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="rgbd">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="rgbd_camera_controller" filename="libgazebo_ros_camera.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>rgbd</cameraName>
    <imageTopicName>rgb/image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>depth/points</pointCloudTopicName>
    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>rgbd_optical_frame</frameName>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

### Depth Camera Noise Modeling

Add realistic noise to depth measurements:

```xml
<camera name="noisy_depth">
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm noise at 1m -->
  </noise>
</camera>
```

## IMU Simulation

Inertial Measurement Units (IMUs) provide crucial information about robot orientation, acceleration, and angular velocity for humanoid balance and navigation.

### Basic IMU Configuration

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.017</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.017</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.017</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</gazebo>
```

### Advanced IMU Configuration

For humanoid robots, IMUs are often placed in multiple locations:

```xml
<!-- IMU in torso for balance -->
<gazebo reference="torso_imu">
  <sensor name="torso_imu_sensor" type="imu">
    <!-- ... IMU configuration ... -->
  </sensor>
</gazebo>

<!-- IMU in head for orientation -->
<gazebo reference="head_imu">
  <sensor name="head_imu_sensor" type="imu">
    <!-- ... IMU configuration ... -->
  </sensor>
</gazebo>

<!-- IMU in feet for ground contact -->
<gazebo reference="left_foot_imu">
  <sensor name="left_foot_imu_sensor" type="imu">
    <!-- ... IMU configuration ... -->
  </sensor>
</gazebo>
```

## Other Sensor Types

### Force/Torque Sensors

Essential for humanoid manipulation and balance:

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
  <plugin name="ft_sensor_plugin" filename="libgazebo_ros_ft_sensor.so">
    <ros>
      <namespace>/left_foot</namespace>
      <remapping>~/wrench:=ft_sensor</remapping>
    </ros>
    <frame_name>left_foot</frame_name>
  </plugin>
</gazebo>
```

### GPS Simulation

For outdoor humanoid robots:

```xml
<gazebo reference="gps_link">
  <sensor name="navsat" type="gps">
    <always_on>true</always_on>
    <update_rate>4</update_rate>
    <gps>
      <position_sensing>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.2</stddev>
        </noise>
      </position_sensing>
    </gps>
  </sensor>
  <plugin name="gps_plugin" filename="libgazebo_ros_gps.so">
    <ros>
      <namespace>/gps</namespace>
      <remapping>~/out:=fix</remapping>
    </ros>
    <frame_name>gps_link</frame_name>
    <update_rate>4.0</update_rate>
  </plugin>
</gazebo>
```

## Sensor Data Pipelines in ROS 2

### Processing LiDAR Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )

        # Publish processed data
        self.publisher = self.create_publisher(
            LaserScan,
            '/lidar/processed_scan',
            10
        )

    def lidar_callback(self, msg):
        # Process LiDAR data for obstacle detection
        processed_scan = self.process_obstacle_detection(msg)
        self.publisher.publish(processed_scan)

    def process_obstacle_detection(self, scan_msg):
        # Example: filter out invalid ranges
        ranges = []
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max:
                ranges.append(r)
            else:
                ranges.append(float('inf'))

        scan_msg.ranges = ranges
        return scan_msg
```

### Processing Camera Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')

        self.bridge = CvBridge()

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Publish processed images
        self.image_pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image (e.g., edge detection)
            processed_image = self.process_image(cv_image)

            # Convert back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header

            # Publish processed image
            self.image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image(self, image):
        # Example: apply Gaussian blur and edge detection
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to 3-channel for visualization
        edge_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return edge_colored
```

## Sensor Fusion

Combine data from multiple sensors for better perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscriptions to multiple sensors
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)

        # Publisher for fused estimate
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot_pose', 10)

        # Store sensor data
        self.imu_data = None
        self.lidar_data = None

    def imu_callback(self, msg):
        self.imu_data = msg
        self.fuse_sensors()

    def lidar_callback(self, msg):
        self.lidar_data = msg
        self.fuse_sensors()

    def fuse_sensors(self):
        if self.imu_data and self.lidar_data:
            # Example: combine IMU orientation with LiDAR position
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            # Use IMU for orientation
            pose_msg.pose.pose.orientation = self.imu_data.orientation

            # Use LiDAR for position estimation (simplified)
            # In reality, this would use SLAM or other algorithms
            pose_msg.pose.pose.position.x = 0.0
            pose_msg.pose.pose.position.y = 0.0
            pose_msg.pose.pose.position.z = 0.0

            self.pose_pub.publish(pose_msg)
```

## Sensor Validation

### Comparing Simulated vs Real Data

To validate sensor simulation accuracy:

```python
# Compare statistical properties of simulated vs real sensor data
def validate_sensor_simulation(simulated_data, real_data):
    # Compare mean values
    sim_mean = np.mean(simulated_data)
    real_mean = np.mean(real_data)
    mean_diff = abs(sim_mean - real_mean)

    # Compare standard deviations
    sim_std = np.std(simulated_data)
    real_std = np.std(real_data)
    std_diff = abs(sim_std - real_std)

    # Compare distributions using Kolmogorov-Smirnov test
    ks_statistic, p_value = scipy.stats.ks_2samp(simulated_data, real_data)

    return {
        'mean_difference': mean_diff,
        'std_difference': std_diff,
        'ks_statistic': ks_statistic,
        'p_value': p_value
    }
```

## Best Practices for Sensor Simulation

### 1. Match Real Sensor Specifications

Configure simulation parameters to match your real sensors:

```xml
<!-- If your real LiDAR has 0.5° angular resolution -->
<horizontal>
  <samples>720</samples>  <!-- For 360° FOV: 360/0.5 = 720 samples -->
  <resolution>1</resolution>
  <min_angle>-3.14159</min_angle>
  <max_angle>3.14159</max_angle>
</horizontal>
```

### 2. Add Realistic Noise Models

Include noise characteristics that match your real sensors:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <!-- Stddev based on manufacturer specs -->
  <stddev>0.015</stddev>  <!-- For LiDAR with ±1.5cm accuracy -->
</noise>
```

### 3. Sensor Placement Considerations

Place sensors similarly to your real robot:

```xml
<!-- IMU near robot's center of mass -->
<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Near CoM -->
</joint>
```

## Troubleshooting Common Issues

### High CPU Usage

- Reduce sensor update rates
- Use simpler collision geometries
- Limit number of active sensors

### Sensor Data Inconsistencies

- Check coordinate frame transformations
- Verify sensor mounting poses
- Ensure proper TF tree setup

### Unrealistic Sensor Readings

- Verify noise parameters
- Check sensor ranges and resolutions
- Validate environmental lighting (for cameras)

## Practice Exercise

1. Create a humanoid robot model with:
   - One LiDAR sensor on the head
   - One depth camera on the head
   - IMUs in torso and feet
   - Force/torque sensors in feet

2. Configure realistic parameters based on commercial sensors
3. Create a ROS 2 node that subscribes to all sensor data
4. Implement a simple sensor fusion algorithm that combines IMU and LiDAR data
5. Validate that sensor data is being published correctly

## Summary

Sensor simulation is essential for creating realistic digital twins of humanoid robots. By properly configuring LiDAR, cameras, IMUs, and other sensors with realistic parameters and noise models, you can create simulations that closely match real-world behavior. This enables comprehensive testing of perception, navigation, and control algorithms before deployment on physical hardware.