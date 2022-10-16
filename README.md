## Autonomous Robots

Course offered in the electrical department at the [UFES](https://www.ufes.br/), by PhD Ricardo Carminati de Mello. This repository includes several ROS2 packages that were developed during the classes.

### Building all packages

If you have a ROS2 environment configured, it might be interesting to create a workspace and add this repository (among other intermediate steps). However, if you have docker installed on your machine and you can build and run the packages proposed here. To do so, just build the docker image:

```bash
docker build -f etc/docker/Dockerfile -t autonomous-robots .
```
### Turtlesim Controller

Using a simple proportional controller to move turtle. You can run the turtle simulation and its controller with:
```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --rm autonomous-robots:latest ros2 launch launch/turtlesim_controller_launch.py
```

Using an action client, you can send the turtle wherever you want with, e.g.:
```bash
docker run -it --rm autonomous-robots:latest ros2 run turtlesim_controller client -x 5 -y 8
```

The turtle controller exposes several configuration parameters, you can check them with:
```bash
docker run -it --rm autonomous-robots:latest ros2 param list
```

To modify the parameter:
```bash
docker run -it --rm autonomous-robots:latest ros2 param set /turtlesim_controller kp_angular 6
```

For more information about each configuration parameter, you can describe each one with:
```bash
docker run -it --rm autonomous-robots:latest ros2 param describe /turtlesim_controller kp_angular
```

### Learning resources

* [Programming Multiple Robots with ROS2](https://osrf.github.io/ros2multirobotbook/), book about multi-robot systems;
* [ROS2 Humble docs](https://docs.ros.org/en/humble/index.html), ROS2 humble distribution documentation page;
* [ros2/example_interfaces](https://github.com/ros2/example_interfaces), github repository with an sample package about creating interfaces;
* [ros2/examples](https://github.com/ros2/examples), github repository with several ROS2 sample packages in C++ and Python;
* [ros2/rclpy](https://github.com/ros2/rclpy), github repository with ROS Client Library for the Python language;
* [ros2/demos](https://github.com/ros2/demos), github repository with several ROS2 demos.