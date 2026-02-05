---
sidebar_position: 2
title: "Writing ROS 2 Nodes with Python"
---

# Writing ROS 2 Nodes with Python

## Learning Objectives

After completing this section, you will be able to:
- Create and structure ROS 2 nodes in Python
- Implement different types of communication patterns using rclpy
- Handle parameters and configuration in Python nodes
- Apply debugging and testing techniques for Python ROS 2 nodes
- Follow best practices for Python node development

## Introduction

Python is one of the most popular languages in the robotics community, thanks to its simplicity, readability, and extensive ecosystem of libraries. The rclpy library provides a Python interface to ROS 2, allowing you to create sophisticated robotic applications with minimal code overhead.

This section will guide you through the process of writing effective ROS 2 nodes in Python, from basic structure to advanced patterns.

## Basic Node Structure

Every ROS 2 Python node follows a similar structure. Let's break down the essential components:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Call the parent class constructor
        super().__init__('node_name')

        # Initialize node components here
        self.get_logger().info('Node initialized successfully')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of your node
    my_node = MyNode()

    # Keep the node running and processing callbacks
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Points:
- Always call `super().__init__('node_name')` in your constructor
- Use `rclpy.spin()` to keep your node active
- Always clean up with `destroy_node()` and `shutdown()`

## Creating Publishers

Publishers allow nodes to send messages to topics. Here's how to create and use them:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create a publisher with topic name, message type, and queue size
        self.publisher = self.create_publisher(String, 'my_topic', 10)

        # Create a timer to periodically publish messages
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds
        self.counter = 0

    def timer_callback(self):
        # Create a message
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the event
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Best Practices:
- Choose appropriate queue sizes based on your application needs
- Use timers for periodic publishing
- Always log important events for debugging

## Creating Subscribers

Subscribers receive messages from topics and trigger callbacks when messages arrive:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create a subscription with topic name, message type, and callback
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.subscription_callback,
            10  # Queue size
        )

        # Prevent unused variable warning
        self.subscription  # Important: keep reference to subscription

    def subscription_callback(self, msg):
        # Process the received message
        self.get_logger().info(f'Received: {msg.data}')

        # You can perform complex processing here
        processed_data = self.process_message(msg.data)

        # Log the processing result
        self.get_logger().info(f'Processed: {processed_data}')

    def process_message(self, data):
        # Example processing function
        return data.upper()

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Best Practices:
- Always maintain a reference to your subscription to prevent garbage collection
- Keep callbacks lightweight; avoid heavy computation in callbacks
- Use separate threads for intensive processing if needed

## Working with Parameters

Parameters allow you to configure your nodes externally:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('message_prefix', 'Hello')
        self.declare_parameter('counter_max', 100)

        # Get parameter values
        self.frequency = self.get_parameter('publish_frequency').value
        self.prefix = self.get_parameter('message_prefix').value
        self.max_count = self.get_parameter('counter_max').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'param_topic', 10)

        # Create timer based on parameter
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        if self.counter >= self.max_count:
            self.get_logger().info('Counter reached maximum, stopping.')
            return

        msg = String()
        msg.data = f'{self.prefix} - Count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Best Practices:
- Always declare parameters before accessing them
- Provide sensible default values
- Consider using parameter callbacks for dynamic reconfiguration

## Service Servers and Clients

### Service Server:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')

        # Create a service server
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Service server started')

    def add_two_ints_callback(self, request, response):
        # Process the request
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')

        # Return the response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')

        # Create a service client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create request object
        self.request = AddTwoInts.Request()

        # Create a timer to periodically call the service
        self.timer = self.create_timer(2.0, self.call_service)

    def call_service(self):
        # Set request parameters
        self.request.a = 42
        self.request.b = 58

        # Call the service asynchronously
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.service_response_callback)

        self.get_logger().info(f'Calling service with {self.request.a} + {self.request.b}')

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Patterns: Composition and Lifecycle

### Composition Nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ComposedNode(Node):
    def __init__(self):
        super().__init__('composed_node')

        # Multiple publishers and subscribers in one node
        self.pub1 = self.create_publisher(String, 'output1', 10)
        self.pub2 = self.create_publisher(String, 'output2', 10)

        self.sub1 = self.create_subscription(String, 'input1', self.callback1, 10)
        self.sub2 = self.create_subscription(String, 'input2', self.callback2, 10)

        self.buffer = {'input1': None, 'input2': None}

    def callback1(self, msg):
        self.buffer['input1'] = msg.data
        self.process_and_publish()

    def callback2(self, msg):
        self.buffer['input2'] = msg.data
        self.process_and_publish()

    def process_and_publish(self):
        if all(self.buffer.values()):
            # Process combined data
            result1 = f'Combined: {self.buffer["input1"]} + {self.buffer["input2"]}'
            result2 = f'Merged: {len(self.buffer["input1"])} + {len(self.buffer["input2"])}'

            # Publish results
            msg1 = String()
            msg1.data = result1
            self.pub1.publish(msg1)

            msg2 = String()
            msg2.data = result2
            self.pub2.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = ComposedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Logging

Proper error handling is crucial for robust nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        self.publisher = self.create_publisher(String, 'safe_topic', 10)
        self.timer = self.create_timer(1.0, self.safe_timer_callback)

    def safe_timer_callback(self):
        try:
            # Potentially risky operation
            result = self.potentially_risky_operation()

            # Publish result
            msg = String()
            msg.data = str(result)
            self.publisher.publish(msg)

        except ValueError as e:
            self.get_logger().error(f'Value error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            self.get_logger().error(traceback.format_exc())

    def potentially_risky_operation(self):
        # Simulate some operation that might fail
        import random
        if random.random() < 0.1:  # 10% chance of error
            raise ValueError("Simulated error for demonstration")
        return "Operation successful"
```

## Debugging Techniques

### 1. Effective Logging:

```python
# Use different log levels appropriately
self.get_logger().debug('Detailed debug information')
self.get_logger().info('General information')
self.get_logger().warn('Warning message')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Fatal error')
```

### 2. Debug Parameters:

```python
# Add debug parameter to enable/disable verbose logging
self.declare_parameter('debug_mode', False)
self.debug_mode = self.get_parameter('debug_mode').value

if self.debug_mode:
    self.get_logger().debug(f'Detailed state: {self.internal_state}')
```

## Testing Your Nodes

### Unit Testing Template:

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from your_package.your_node import YourNode

class TestYourNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = YourNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        self.assertIsNotNone(self.node)

    def test_parameter_values(self):
        # Test parameter values
        pass

if __name__ == '__main__':
    unittest.main()
```

## Practice Exercise

Create a Python node that implements the following functionality:
1. Subscribes to a topic that receives temperature readings
2. Applies a simple filter (e.g., moving average) to the data
3. Publishes the filtered temperature to another topic
4. Provides a service that returns the current average temperature
5. Uses parameters to configure the filter window size

Test your node with simulated temperature data.

## Summary

Writing ROS 2 nodes in Python with rclpy provides a powerful yet accessible way to create sophisticated robotic applications. By following proper structure, implementing error handling, and using parameters effectively, you can create robust and maintainable nodes that integrate seamlessly with the broader ROS 2 ecosystem.