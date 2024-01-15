## Turtlebot3 Controller

Position controller to move the [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) to a specified location.

### About

Just like [turtlesim_controller](../turtlesim_controller/) package, this package implements a simple position controller to the [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3). Given a goal position $(x_g, y_g)^T$, we want to take the robot from an arbitrary position $(x, y)^T$ to the goal. To do so, we need to eliminate the position error $(\tilde{x},\tilde{y})=(x_g - x, y_g - y)^T$. We can also define the problem using polar coordinates, where the position error is given by:

$$ \begin{pmatrix}p \\\ \alpha \end{pmatrix} = \begin{pmatrix} \sqrt{\tilde{x}^2 + \tilde{y}^2} \\\ \arctan(\frac{\tilde{y}}{\tilde{x}}) - \Psi \end{pmatrix} $$

Where $p$ is the Euclidean distance to the desired point, $\Psi$ is the robot orientation and $\alpha$ is the angle between the robot's current orientation and the vector $p$. Then, we implement a simple control law to see if we can control the robot:

$$ \begin{pmatrix}v \\\ \omega \end{pmatrix} = \begin{pmatrix} k_{v}\tanh(p) \\\ k_{\omega}\alpha \end{pmatrix} $$

Where $k_{v}$ is a positive constant that represents the maximum linear speed and $k_{\omega}$ is a positive constant that represents the maximum angular speed. So, we developed an [action server](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) following these mathematical formulations. The task of moving the turtlebot3 until reaching a final goal is a long running task and actions are the proper way of handling such situations in ROS. You can view the source code of the action server at [turtlesim_controller/controller.py](src/controller.cpp).

### Demo

Next, it's a recording of a working demonstration. Note that we just sent the turtlebot3 some random positions to go.

<p align="center">
    <img src="images/demo.gif?raw=true" width=1200px/>
</p>
<p align="center">
    <em>Demo</em>
</p>

### Building and runinng with ROS2

This repository is organized as a ROS2 workspace, so just clone it:
```bash
git clone https://github.com/luizcarloscf/autonomous-robots.git
cd autonomous-robots/
```

Then, build this package with:
```bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select=turtlebot3_controller
source install/setup.bash
```

Finally, you can launch [Gazebo](https://gazebosim.org/home) with turtlebot3 in an empty world:
```bash
ros2 launch turtlebot3_controller turtlebot3_controller_launch.py
```

The above launch script also starts the turtlebot3 controller. You can test it by opening another terminal and executing (just remenber to source the workspace in this new terminal):
```bash
source install/setup.bash
ros2 action send_goal /task turtlebot3_controller/action/Task "{x: [1.0,2.0,-1.0], y: [1.0,2.0,-1.0]}"
```

#### Running RO2 with [Docker](https://www.docker.com/)

Not really safe, but it works.
```bash
sudo xhost +local:root
```

If you have [Docker](https://www.docker.com/) installed, you dont need to build anything. Just pull our image and test it.
```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri/ luizcarloscf/autonomous-robots:main /bin/bash
```

The first thing you'll need to do is run the gazebo inside the container:
```bash
gazebo
```

I haven't figured out why yet, but the first launch of the gazebo inside the container takes a long time. After the gazebo has opened the first time, you can close it and run the package launch:
```bash
ros2 launch turtlebot3_controller turtlebot3_controller_launch.py
```

If you want to control the robots, just open a new container in the same network and run the action client node:
```bash
docker run -it luizcarloscf/autonomous-robots:main /bin/bash
ros2 action send_goal /task turtlebot3_controller/action/Task "{x: [1.0,2.0,-1.0], y: [1.0,2.0,-1.0]}" 
```