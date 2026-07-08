FROM ros:jazzy

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-rtabmap-ros && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]
