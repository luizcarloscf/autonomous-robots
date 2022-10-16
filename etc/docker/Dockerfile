FROM osrf/ros:humble-desktop-full

# Change the default shell to bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-pip

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install setuptools==58.2.0

WORKDIR /opt/ros2_ws
COPY src/ /opt/ros2_ws/src/
COPY launch/ /opt/ros2_ws/launch/


RUN source /opt/ros/humble/setup.bash && \
    colcon build

COPY /etc/scripts/ros2_ws_entrypoint.sh /ros2_ws_entrypoint.sh
ENTRYPOINT [ "/ros2_ws_entrypoint.sh" ]