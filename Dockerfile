ARG ROS_DISTRO="melodic"
ARG ARCH="amd64"
FROM --platform=linux/$ARCH ros:$ROS_DISTRO

FROM ros:$ROS_DISTRO

FROM osrf/ros:$ROS_DISTRO-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-pip \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-joint-state-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-message-to-tf \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros-control \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-xacro \
    tmux

# Copy repository into ROS workspace
WORKDIR /home/root/kaa_holo_mahina_ws/src
COPY src .

# Setup and build ROS workspace
WORKDIR /home/root/kaa_holo_mahina_ws/
RUN source /opt/ros/$ROS_DISTRO/setup.bash && catkin build -j4
RUN echo 'source /home/root/kaa_holo_mahina_ws/devel/setup.bash' >> /home/root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]