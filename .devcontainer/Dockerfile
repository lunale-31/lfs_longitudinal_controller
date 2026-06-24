# Taking the ROS2 image from open-source robotics foundation 
FROM osrf/ros:humble-desktop

# Running installation commands of necessary things like foxglove and colcon 
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-foxglove-bridge \
    ros-humble-plotjuggler-ros \
    git \
    && rm -rf /var/lib/apt/lists/*

# Match the automatic VS Code workspaces path
WORKDIR /workspaces/controller_ws

# Similar to bashrc commands where we need to source the ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspaces/controller_ws/install/setup.bash ]; then source /workspaces/controller_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]