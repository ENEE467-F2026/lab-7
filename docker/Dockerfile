############################################################
# Base Image for the Hardware Lab: UR3e + Robotiq Hand-E
############################################################
ARG ROS_DISTRO=humble
ARG HOME=/home/robot
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-c"]

# Core dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    autoconf automake bash-completion build-essential cmake make pkg-config g++ git curl wget unzip \
    ca-certificates software-properties-common python3-pip python3-apt sudo \
    python3-rosdep python3-vcstool python3-colcon-common-extensions \
    lsb-release iproute2 iputils-ping net-tools socat \
    mc nano vim tree tmux tzdata screen less xclip figlet toilet \
    libxext-dev libx11-dev x11proto-gl-dev libgl1-mesa-dev libglu1-mesa-dev mesa-utils \
    libgtk-3-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libusb-1.0-0-dev libusb-1.0-0 libssl-dev libudev-dev udev libudev1 libmodbus-dev \
    hwloc libmkl-dev libtbb-dev \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-pcl-* \
    libpcl-dev libboost-dev libblas-dev liblapack-dev python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip install ipython roboticstoolbox-python==1.1.0 python-fcl==0.7.0.8 transforms3d==0.4.2 scikit-image scikit-learn 

# Librealsense setup
ARG LIBREALSENSE_TAG=v2.57.3
RUN git clone --depth 1 --branch ${LIBREALSENSE_TAG} https://github.com/IntelRealSense/librealsense.git /opt/librealsense \
 && cd /opt/librealsense && mkdir build && cd build \
 && cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release \
 && make -j"$(nproc)" && make install \
 && install -m 644 /opt/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules || true

RUN python3 -m pip install pyrealsense2 --no-cache-dir --ignore-installed  \
 || (cd /opt/librealsense && python3 -m pip install . --no-cache-dir --ignore-installed )

# MoveIt, UR Drivers, ROS 2 control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-moveit* \
    ros-${ROS_DISTRO}-ur* \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# rosdep
RUN sudo rosdep fix-permissions && \
    sudo rosdep init 2>/dev/null || true && \
    rosdep update

# UID/GID
ARG userid=1002
ARG groupid=1002
RUN groupadd -g ${groupid} -o robot || true; \
    useradd -ms /bin/bash --uid ${userid} -g ${groupid} --groups sudo,video robot; \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER robot
ENV HOME=/home/robot
WORKDIR /home/robot

# Workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && mkdir -p ~/ros2_ws/src

# code-server
RUN curl -fsSL https://code-server.dev/install.sh | sh

# Shell env
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc && \
    echo "alias clc=clear" >> ~/.bashrc && \
    echo "clear" >> ~/.bashrc && \
    echo "toilet -f smblock --filter border:metal -w 24 ' ENEE467   LAB 7   '" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-i"]