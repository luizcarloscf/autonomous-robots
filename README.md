## Autonomous Robots

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg?style=flat-square)](https://docs.ros.org/en/humble/index.html)
[![Docker build](https://img.shields.io/github/actions/workflow/status/luizcarloscf/autonomous-robots/docker.yml?style=flat-square)](https://github.com/luizcarloscf/autonomous-robots/actions)
[![Docker image size](https://img.shields.io/docker/image-size/luizcarloscf/autonomous-robots?sort=semver&style=flat-square)](https://hub.docker.com/repository/docker/luizcarloscf/autonomous-robots)
[![Docker image tag](https://img.shields.io/docker/v/luizcarloscf/autonomous-robots?sort=semver&style=flat-square)](https://hub.docker.com/repository/docker/luizcarloscf/autonomous-robots)
[![Docker pulls](https://img.shields.io/docker/pulls/luizcarloscf/autonomous-robots?style=flat-square)](https://hub.docker.com/repository/docker/luizcarloscf/autonomous-robots)
[![License](https://img.shields.io/github/license/luizcarloscf/autonomous-robots?style=flat-square)](https://github.com/luizcarloscf/autonomous-robots/blob/main/LICENSE)

Course offered in the electrical department at the [UFES](https://www.ufes.br/), by PhD Ricardo Carminati de Mello. This repository includes several ROS2 packages that were developed during the classes.

### Packages

During the lab classes, several packages were developed. Below there is a description of each one, for more details see the README.md specific to each package.

* [turtlesim_controller](src/turtlesim_controller/): controller to move the turtle to a specified location;
* [turtlesim_interfaces](src/turtlesim_interfaces/): interfaces to the [turtlesim_controller](src/turtlesim_controller/) package;

### Building all packages

If you have a ROS2 environment configured, it might be interesting to create a workspace and add this repository (among other intermediate steps). However, if you have docker installed on your machine and you can build and run all the packages proposed here. To do so, just build the docker image with:

```bash
docker build -f etc/docker/Dockerfile -t autonomous-robots .
```

It is also available all packages already built in a docker image (with all dependencies as well) for you to just spin. Check the repository at dockerhub and choose the image tag you see fit, [luizcarloscf/autonomous-robots](https://hub.docker.com/r/luizcarloscf/autonomous-robots).


### Learning resources

* [Programming Multiple Robots with ROS2](https://osrf.github.io/ros2multirobotbook/), book about multi-robot systems;
* [ROS2 Humble docs](https://docs.ros.org/en/humble/index.html), ROS2 humble distribution documentation page;
* [ros2/example_interfaces](https://github.com/ros2/example_interfaces), github repository with an sample package about creating interfaces;
* [ros2/examples](https://github.com/ros2/examples), github repository with several ROS2 sample packages in C++ and Python;
* [ros2/rclpy](https://github.com/ros2/rclpy), github repository with ROS Client Library for the Python language;
* [ros2/demos](https://github.com/ros2/demos), github repository with several ROS2 demos.