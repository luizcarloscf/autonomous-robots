## Autonomous Robots

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg?style=flat-square)](https://docs.ros.org/en/humble/index.html)
[![Docker build](https://img.shields.io/github/actions/workflow/status/luizcarloscf/autonomous-robots/docker.yml?style=flat-square)](https://github.com/luizcarloscf/autonomous-robots/actions)
[![Docker image size](https://img.shields.io/docker/image-size/luizcarloscf/autonomous-robots?sort=semver&style=flat-square)](https://hub.docker.com/repository/docker/luizcarloscf/autonomous-robots)
[![Docker image tag](https://img.shields.io/docker/v/luizcarloscf/autonomous-robots?sort=semver&style=flat-square)](https://hub.docker.com/repository/docker/luizcarloscf/autonomous-robots)
[![Docker pulls](https://img.shields.io/docker/pulls/luizcarloscf/autonomous-robots?style=flat-square)](https://hub.docker.com/repository/docker/luizcarloscf/autonomous-robots)
[![License](https://img.shields.io/github/license/luizcarloscf/autonomous-robots?style=flat-square)](https://github.com/luizcarloscf/autonomous-robots/blob/main/LICENSE)

This repository includes several packages designed for learning and practicing [ROS] (Robotics Operational System). It all started with a course presented in the [Electrical Engineering] department at the [UFES], by [PhD Ricardo Carminati de Mello]. 

> The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers and state-of-the-art algorithms to powerful developer tools, ROS has the open source tools you need for your next robotics project. 

### ROS Packages

Throughout the laboratory classes, we developed some packages. Check out brief descriptions of each below, and for in-depth information, dive into the respective README.md of each package.

* [turtlesim_controller](turtlesim_controller/): position controller to move a turtle simulation;

* [turtlesim_interfaces](turtlesim_interfaces/): interfaces to the [turtlesim_controller](turtlesim_controller/) package;

* [turtlebot3_controller](turtlebot3_controller): position controller to move [Turtlebot3] simulation in [Gazebo].

### Building all packages

If you have a ROS2 environment configured,
```bash
cd ~/ros_ws/src
git clone https://github.com/luizcarloscf/autonomous-robots.git
cd ~/ros_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
```

However, if you have [Docker] installed on your machine and you can use it to build all packages. To do so, just build the docker image with:
```bash
git clone https://github.com/luizcarloscf/autonomous-robots.git
cd autonomous-robots/
docker build -t autonomous-robots .
```

It is also available all packages already built in a docker image (with all dependencies as well) for you to just spin. Check the repository at dockerhub and choose the image tag you see fit, [luizcarloscf/autonomous-robots].

### Learning resources

* [Programming Multiple Robots with ROS2]: book about multi-robot systems;
* [ROS2 Humble docs]: ROS2 humble distribution documentation page;
* [ros2/example_interfaces]: github repository with a sample package about creating interfaces;
* [ros2/examples]: github repository with several ROS2 sample packages in C++ and Python;
* [ros2/rclpy]: github repository with ROS Client Library for the Python language;
* [ros2/demos]: github repository with several ROS2 demos.

<!-- Links -->
[Electrical Engineering]: https://ele.ufes.br/
[UFES]: https://www.ufes.br/
[ROS]: https://docs.ros.org/en/humble/index.html
[PhD Ricardo Carminati de Mello]: http://lattes.cnpq.br/1569638571582691
[Turtlesim]: https://index.ros.org/p/turtlesim/github-ros-ros_tutorials/
[Turtlebot3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
[Gazebo]: https://gazebosim.org/
[Docker]: https://www.docker.com/
[luizcarloscf/autonomous-robots]: https://hub.docker.com/r/luizcarloscf/autonomous-robots

<!-- Learning resources -->
[Programming Multiple Robots with ROS2]: https://osrf.github.io/ros2multirobotbook/
[ROS2 Humble docs]: https://docs.ros.org/en/humble/index.html
[ros2/example_interfaces]: https://github.com/ros2/example_interfaces
[ros2/examples]: https://github.com/ros2/examples
[ros2/rclpy]: https://github.com/ros2/rclpy
[ros2/demos]: https://github.com/ros2/demos