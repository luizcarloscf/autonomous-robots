FROM osrf/ros:humble-desktop-full

# Change the default shell to bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-pip

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

WORKDIR /opt/ros2_ws
COPY src/ /opt/ros2_ws/src/

RUN source /opt/ros/humble/setup.bash && \
    colcon build

COPY /etc/scripts/ros2_ws_entrypoint.sh /ros2_ws_entrypoint.sh
ENTRYPOINT [ "/ros2_ws_entrypoint.sh" ]