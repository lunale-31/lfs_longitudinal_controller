FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-foxglove-bridge \
    git \
    && rm -rf /var/lib/apt/lists/*

# Match the automatic VS Code workspaces path
WORKDIR /workspaces/controller_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspaces/controller_ws/install/setup.bash ]; then source /workspaces/controller_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]