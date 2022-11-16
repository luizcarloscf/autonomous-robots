## Turtlesim Interfaces

Interfaces for the [turtlesim_controller](../turtlesim_controller/) package.

### About

As highlighted in the ROS2 documentation about [interfaces](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html),

> ROS applications typically communicate through interfaces of one of three types: messages, services and actions. ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

In this package, an action interface is implemented for the action server of[turtlesim_controller](../turtlesim_controller/) package to use. You can view the interface and its attributes by running:

```bash
$ ros2 interface show turtlesim_interfaces/action/TurtleTask

# Goal point
float32 x
float32 y
---
# Final result after the system converges
float32 x
float32 y
---
# Feedback of current position untils it converges
float32 x
float32 y
```