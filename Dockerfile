ARG ROS_DISTRO=jazzy
ARG ROS_ARCH=amd64

# linux/amd64 only for now
#https://hub.docker.com/layers/osrf/ros/${ROS_DISTRO}-desktop/images/sha256-742948bc521573ff962f5a7f084ba1562a319e547c3938603f8dff5d33d3466e?context=explore
FROM ${ROS_ARCH}/ros:${ROS_DISTRO}


# Used for install gcc-9 and libstdc++6 required for GLIBCXX_3.4.26 for the ArenaSDK
RUN apt-get update --allow-insecure-repositories && \
    apt-get install software-properties-common -y --allow-unauthenticated && \
    add-apt-repository ppa:ubuntu-toolchain-r/test -y

# Install packages
RUN apt-get update --allow-insecure-repositories \
    && apt-get install -y --no-install-recommends --allow-unauthenticated \
    git cmake curl wget unzip ninja-build ca-certificates vim xauth \
    gnupg gnupg2 lsb-release openjdk-8-jdk\
    python3-dev python3-pip python3-numpy python3-matplotlib python3-empy python3-tk python3-full \
    #    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    #    ros-${ROS_DISTRO}-ament-cmake \
    gcc-9 libstdc++6 \
    && apt-get upgrade -y

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-cyclonedds
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add Kitware APT repository for latest CMake
RUN apt-get update --allow-insecure-repositories && apt-get install -y --no-install-recommends --allow-unauthenticated \
    apt-transport-https ca-certificates gnupg software-properties-common && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - && \
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' && \
    apt-get update && apt-get install -y cmake && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libibverbs-dev \
    librdmacm-dev \
    libmlx5-1 \
    && rm -rf /var/lib/apt/lists/*

RUN cmake --version
# ARGS might want to change ---------------------------------------------------

# ArenaSDK tar file on parent the host relative to the build context; it must contain the ArenaSDK .tar.gz file
ARG arenasdk_root_on_host=./resources/ArenaSDK/linux64 

# location of arenasdk parent folder in the container; this is where the tar file would be executed
# Note:
# if arenasdk_parent is not in root "/", the ${arenasdk_root} must be updated
ARG arenasdk_parent=/

# when the tar gets excuted in ${arenasdk_parent}, it create a folder ,arenasdk root,
# with a CONSTANT name that can not be modified which is "ArenaSDK_Linux_x64"
ARG arenasdk_root=${arenasdk_parent}/ArenaSDK_Linux_x64

# arena_api-<x.x.x.>.whl file parent folder on the host relative to the build context
ARG arena_api_root_on_host=./resources/arena_api

# location of arena_api parent folder in the container; this is where the whl file would be copied
ARG arena_api_parent=/arena_api


# install ArenaSDK ------------------------------------------------------------

# copy binraies from host into root
COPY ${arenasdk_root_on_host}/ArenaSDK_v0.1.91_Linux_x64.tar.gz ${arenasdk_parent}

# copy isntallation script from host into ArenSDK root
ADD ${arenasdk_root_on_host}/*.sh ${arenasdk_root}

RUN tar -xvf /ArenaSDK_v0.1.91_Linux_x64.tar.gz -C ${arenasdk_parent}

WORKDIR ${arenasdk_root}

RUN /bin/bash -c "sh Arena_SDK_Linux_x64.conf"

RUN /bin/bash -c "export ARENA_ROOT=${arenasdk_root}"

# install arena_api whl -------------------------------------------------------

# copy whl file from host
ADD ${arena_api_root_on_host}/*.whl ${arena_api_parent}/

# install via pip3 all whl files in the arena_api parent dir
RUN for whl_package in `ls ${arena_api_parent}/*.whl`; do pip3 install --break-system-packages $whl_package; done

# setup workspace -------------------------------------------------------------

# setup entrypoint
# entry point script:
#  - installs package's depens
#  - builds workspace and install it symlink way 
#  - source workspace
ADD ./arena_camera_ros_entrypoint.sh /
#RUN chmod 777 /arena_camera_ros_entrypoint.sh


WORKDIR /arena_camera_ros2/ros2_ws
COPY ros2_ws/src ros2_ws/src
RUN ["/bin/bash", "-c", " source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"]


ENTRYPOINT [ "/bin/bash", "-c", " source /opt/ros/${ROS_DISTRO}/setup.bash && source ./install/setup.bash && ros2 run arena_camera_node start --ros-args -p binning_selector:=Sensor -p width:=2280 -p height:=450 -p offset_x:=300 -p offset_y:=300" ]

