## Turtlesim Controller

Go to Goal controller for the turtlesim simulation.

### About

Here, the turtle represents a robot that we want to control. Given a goal position $(x_g, y_g)^T$, we want to take the robot from an arbitrary position $(x, y)^T$ to the goal. To do so, we need to eliminate the position error $(\tilde{x},\tilde{y})=(x_g - x, y_g - y)^T$. We can also define the problem using polar coordinates, where the position error is given by:

$$ \begin{pmatrix}p \\ \alpha \end{pmatrix} = \begin{pmatrix} \sqrt{\tilde{x}^2 + \tilde{y}^2} \\ \arctan(\frac{\tilde{y}}{\tilde{x}}) - \Psi \end{pmatrix} $$

Where $p$ is the Euclidean distance to the desired point, $\Psi$ is the robot orientation and $\alpha$ is the angle between the robot's current orientation and the vector $p$. Then, we implement a simple control law to see if we can control the robot:

$$ \begin{pmatrix}v \\ \omega \end{pmatrix} = \begin{pmatrix} k_{v}\tanh(p) \\ k_{\omega}\alpha \end{pmatrix} $$

Where $k_{v}$ is a positive constant that represents the maximum linear speed and $k_{\omega}$ is a positive constant that represents the maximum angular speed.

### Demo

Below, its a recording of a working demonstration. Note that we just sent the turtle some random positions to go.

<p align="center">
    <img src="images/1.gif" />
</p>
<p align="center">
    <em>Demo</em>
</p>

### Running

You can run the turtle simulation and its controller with:
```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --rm luizcarloscf/autonomous-robots:main ros2 launch turtlesim_controller turtlesim_controller_launch.py
```

Using an action client, you can send the turtle wherever you want with, e.g.:
```bash
docker run -it --rm luizcarloscf/autonomous-robots:main ros2 run turtlesim_controller client -x 5 -y 8
```

The turtle controller exposes several configuration parameters, you can check them with:
```bash
docker run -it --rm luizcarloscf/autonomous-robots:main ros2 param list
```

To modify the parameter:
```bash
docker run -it --rm luizcarloscf/autonomous-robots:main ros2 param set /turtlesim_controller kp_angular 6
```

For more information about each configuration parameter, you can describe each one with:
```bash
docker run -it --rm luizcarloscf/autonomous-robots:main ros2 param describe /turtlesim_controller kp_angular
```
