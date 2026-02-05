---
sidebar_position: 2
title: "Nodes, Topics, Services, and Actions"
---

# Nodes, Topics, Services, and Actions

## Learning Objectives

After completing this section, you will be able to:
- Define and distinguish between nodes, topics, services, and actions
- Understand the communication patterns in ROS 2
- Choose the appropriate communication method for different scenarios
- Implement basic examples of each communication type

## Introduction

Communication is the backbone of any distributed robotic system. In ROS 2, there are four primary methods for nodes to communicate with each other: nodes, topics, services, and actions. Understanding these concepts is crucial for developing effective robotic applications.

## Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 program. Each node performs a specific task and communicates with other nodes to achieve complex behaviors.

### Key Characteristics of Nodes:
- Encapsulate functionality in a single process
- Communicate with other nodes through topics, services, or actions
- Managed by the ROS 2 runtime system
- Identified by unique names within the ROS graph

### Example Node Structure:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Topics

**Topics** enable asynchronous communication between nodes using a publish-subscribe model. Publishers send messages to a topic, and subscribers receive messages from that topic. This creates a decoupled communication system where publishers and subscribers don't need to know about each other.

### Key Characteristics of Topics:
- Unidirectional data flow (publisher to subscriber)
- Asynchronous communication
- Multiple publishers and subscribers can use the same topic
- Best for streaming data or continuous sensor readings

### Topic Communication Example:

```python
# Publisher
publisher = node.create_publisher(String, 'chatter', 10)

# Subscriber
subscription = node.create_subscription(
    String,
    'chatter',
    callback_function,
    10)
```

## Services

**Services** enable synchronous request-response communication between nodes. A service client sends a request to a service server, which processes the request and sends back a response. This is similar to a traditional client-server model.

### Key Characteristics of Services:
- Synchronous communication
- Request-response pattern
- Only one server per service (though multiple clients can call it)
- Best for operations that have a clear beginning and end

### Service Example:

```python
# Service Server
service = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

# Service Client
client = node.create_client(AddTwoInts, 'add_two_ints')
```

## Actions

**Actions** are a more advanced form of communication that combines aspects of both topics and services. Actions are designed for long-running tasks that may need to be preempted and provide continuous feedback during execution.

### Key Characteristics of Actions:
- Long-running tasks
- Support for cancellation/preemption
- Continuous feedback during execution
- Goal, result, and feedback messages
- Best for tasks like navigation, manipulation, or calibration

### Action Example:

```python
# Action Server
action_server = ActionServer(
    node,
    Fibonacci,
    'fibonacci',
    execute_callback)

# Action Client
action_client = ActionClient(node, Fibonacci, 'fibonacci')
```

## Comparison of Communication Methods

| Method | Communication Style | Use Case | Example |
|--------|-------------------|----------|---------|
| Topics | Publish-Subscribe (Async) | Streaming data | Sensor readings |
| Services | Request-Response (Sync) | Discrete operations | Calculations |
| Actions | Goal-Based (Long-running) | Extended tasks | Navigation |

## When to Use Each Method

- **Use Topics** for continuous data streams like sensor data, camera feeds, or robot state updates.
- **Use Services** for operations that have a clear input/output relationship, like calculating distances or transforming coordinates.
- **Use Actions** for tasks that take time to complete and may need monitoring or interruption, like robot navigation or arm movements.

## Practice Exercise

Create a simple publisher node that publishes messages to a topic called "my_topic" every 2 seconds. Then create a subscriber node that listens to this topic and prints the received messages.

## Summary

Understanding the different communication methods in ROS 2 is essential for designing effective robotic systems. Each method serves a specific purpose and choosing the right one for your use case will make your robot applications more robust and efficient.