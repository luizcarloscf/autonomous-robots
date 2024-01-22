## Turtlesim Controller

Position controller to move the turtle simulatoin. [Turtlesim] is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level to give you an idea of what you will do with a real robot or a robot simulation later on;

### About

Here, the turtle represents a robot that we want to control. Given a goal position $(x_g, y_g)^T$, we want to take the robot from an arbitrary position $(x, y)^T$ to the goal. To do so, we need to eliminate the position error $(\tilde{x},\tilde{y})=(x_g - x, y_g - y)^T$. We can also define the problem using polar coordinates, where the position error is given by:

$$ \begin{pmatrix}p \\\ \alpha \end{pmatrix} = \begin{pmatrix} \sqrt{\tilde{x}^2 + \tilde{y}^2} \\\ \arctan(\frac{\tilde{y}}{\tilde{x}}) - \Psi \end{pmatrix} $$

Where $p$ is the euclidean distance to the desired point, $\Psi$ is the robot orientation and $\alpha$ is the angle between the robot's current orientation and the vector $p$. Then, we implement a simple control law to see if we can control the robot:

$$ \begin{pmatrix}v \\\ \omega \end{pmatrix} = \begin{pmatrix} k_{v}\tanh(p) \\\ k_{\omega}\alpha \end{pmatrix} $$

Where $k_{v}$ is a positive constant that represents the maximum linear speed and $k_{\omega}$ is a positive constant that represents the maximum angular speed. So, we developed an [action server] following these mathematical formulations. The task of moving the turtle until reaching a final goal is a long running task and actions are the proper way of handling such situations in ROS. You can view the source code of the action server at [turtlesim_controller/controller.py](turtlesim_controller/controller.py). We also provide an action client at [turtlesim_controller/controller_client.py](turtlesim_controller/controller_client.py).

Note that to send the robot to a goal position, receive feedback and the final result, it is necessary to implement an interface for the action server. Check the [turtlesim_interfaces](../turtlesim_interfaces/) package for more details.

### Demo

Below, its a recording of a working demonstration. Note that we just sent the turtle some random positions to go.

<p align="center">
    <img src="images/1.gif?raw=true" />
</p>
<p align="center">
    <em>Demo</em>
</p>

### Building and runinng

Just clone this project in your ROS2 workspace:
```bash
cd ~/ros_ws/src
git clone https://github.com/luizcarloscf/autonomous-robots.git
cd ~/ros_ws
```

Then, build this package with:
```bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-up-to=turtlesim_controller
source install/setup.bash
```

Finally, you can launch with:
```bash
ros2 launch turtlesim_controller turtlesim_controller_launch.py
```

The above launch script also starts the turtlesim controller. You can test it by opening another terminal and executing (just remenber to source the workspace in this new terminal):
```bash
source install/setup.bash
ros2 action send_goal /turtletask turtlesim_interfaces/action/TurtleTask "{x: 1.0, y: 10.0}"
```

The turtle controller exposes several configuration [parameters], you can check them with:

```bash
ros2 param list
```

To modify the parameter:
```bash
ros2 param set /turtlesim_controller kp_angular 6
```

For more information about each configuration parameter, you can describe each one with:
```bash
ros2 param describe /turtlesim_controller kp_angular
```

### Running with [Docker]

You can run the turtle simulation and its controller with [Docker] and a [launch script](launch/turtlesim_controller_launch.py):

```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --rm luizcarloscf/autonomous-robots:main ros2 launch turtlesim_controller turtlesim_controller_launch.py
```

This program will run the turtle simulation and also run its controller. Using an [action client](turtlesim_controller/controller_client.py), you can send the turtle wherever you want with, e.g.:

```bash
docker run -it --rm luizcarloscf/autonomous-robots:main ros2 run turtlesim_controller client -x 5 -y 8
```

[Turtlesim]: https://index.ros.org/p/turtlesim/github-ros-ros_tutorials/
[action server]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
[Docker]: https://www.docker.com/
[parameters]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html