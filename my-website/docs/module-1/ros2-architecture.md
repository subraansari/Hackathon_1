---
sidebar_position: 3
title: "ROS 2 Architecture"
---

# ROS 2 Architecture

## Learning Objectives

After completing this section, you will be able to:
- Describe the main components of the ROS 2 architecture
- Explain the role of DDS in ROS 2 communication
- Understand the lifecycle of a ROS 2 system
- Identify different QoS policies and their use cases

## Introduction

The architecture of ROS 2 represents a significant evolution from ROS 1, primarily driven by the need for improved security, real-time capabilities, and commercial viability. At its core, ROS 2 leverages Data Distribution Service (DDS) as its underlying communication middleware, providing a robust foundation for distributed robotic systems.

## Architecture Overview

ROS 2 architecture is built around several key components that work together to provide a comprehensive framework for robot development:

### 1. Client Libraries

ROS 2 provides client libraries that abstract the complexity of DDS communication:

- **rclcpp**: C++ client library
- **rclpy**: Python client library
- **rcl**: C client library (common layer)
- **rclc**: C client library for microcontrollers

### 2. DDS/RMW Layer

The DDS (Data Distribution Service) and RMW (ROS Middleware) layer is the core of ROS 2's communication system. It provides:

- Discovery mechanisms
- Message serialization
- Quality of Service (QoS) policies
- Network transport protocols

### 3. ROS Graph

The ROS graph represents all nodes and their connections in the system. Each node is assigned a unique identifier, and communication paths between nodes form the graph structure.

## DDS in ROS 2

### What is DDS?

DDS is an internationally standardized (OMG) machine-to-machine connectivity framework for real-time, scalable, dependable, and high-performance applications. In ROS 2, DDS serves as the underlying communication middleware.

### Key DDS Concepts

- **Domain**: A communication space where participants can discover each other
- **Participant**: An entity that participates in a DDS domain
- **Topic**: A named data channel for communication
- **Publisher/Subscriber**: Entities that send or receive data on topics
- **DataWriter/DataReader**: Lower-level entities that handle actual data transmission

### DDS Implementations

ROS 2 supports multiple DDS implementations:

- **Fast DDS** (formerly Fast RTPS): Default implementation
- **Cyclone DDS**: Lightweight and efficient
- **RTI Connext DDS**: Commercial implementation
- **OpenSplice DDS**: Open-source implementation

## Quality of Service (QoS) Policies

QoS policies allow fine-tuning of communication behavior to meet specific application requirements:

### Reliability Policy
- **Reliable**: All messages are guaranteed to be delivered
- **Best Effort**: Messages may be lost, but delivery is faster

### Durability Policy
- **Transient Local**: Late-joining subscribers receive historical data
- **Volatile**: Only future messages are sent to new subscribers

### History Policy
- **Keep Last**: Maintain only the most recent N samples
- **Keep All**: Maintain all samples until resources are exhausted

### Deadline Policy
- Specifies maximum interval between data samples

### Lifespan Policy
- Defines how long published data remains valid

## ROS 2 Launch System

The launch system in ROS 2 provides enhanced capabilities compared to ROS 1:

### Features
- Cross-platform compatibility (Linux, Windows, macOS)
- Python-based launch files
- Composable nodes (components within a single process)
- Event-driven execution

### Example Launch File:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_node'
        )
    ])
```

## Lifecycle Nodes

ROS 2 introduces lifecycle nodes for managing complex node states:

### States
- **Unconfigured**: Node exists but is not configured
- **Inactive**: Node is configured but not active
- **Active**: Node is running and operational
- **Finalized**: Node is shutting down

### Benefits
- Controlled startup and shutdown sequences
- Resource management
- Dynamic reconfiguration

## Parameter System

ROS 2 provides a more robust parameter system:

### Features
- Hierarchical parameter namespaces
- Parameter validation
- Parameter callbacks
- Remote parameter access

### Example Parameter Declaration:

```python
self.declare_parameter('my_param', 'default_value')
```

## Security Framework

ROS 2 includes built-in security features:

### Components
- Authentication: Verifying identity of nodes
- Access Control: Defining who can access what
- Encryption: Protecting data in transit

## Practical Architecture Example

Consider a mobile robot with the following nodes:

```
Camera Node → Image Processing Node → Navigation Node
     ↓              ↓                     ↓
Lidar Node → Perception Fusion → Motion Controller
```

Each node would:
- Subscribe to relevant topics
- Publish processed data
- Use appropriate QoS policies based on real-time requirements
- Potentially use services for configuration requests

## Best Practices

1. **Choose appropriate QoS policies** based on your application's requirements
2. **Use namespaces** to organize related nodes and topics
3. **Design for fault tolerance** by considering what happens when nodes fail
4. **Monitor system resources** to ensure performance requirements are met
5. **Plan for security** from the beginning of your design

## Summary

ROS 2's architecture provides a solid foundation for building robust, scalable, and secure robotic systems. Understanding these architectural concepts is crucial for effective ROS 2 development and will help you make informed decisions about your robot's design and implementation.