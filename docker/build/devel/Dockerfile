################################################################################
# Source image                                                                 #
################################################################################

FROM ubuntu:22.04

################################################################################
# Build arguments                                                              #
################################################################################

ARG BUILD_TYPE=RelWithDebInfo
ARG DEBIAN_FRONTEND=noninteractive
ARG LOCALE=en_US
ARG ROS_DISTRIBUTION=humble
ARG USER=robot

################################################################################
# Install the ROS distribution                                                 #
################################################################################

# Install ROS dependencies and set up locales.
RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
    curl \
    locales \
    software-properties-common && \
    locale-gen ${LOCALE} ${LOCALE}.UTF-8 && \
    update-locale LC_ALL=${LOCALE}.UTF-8 LANG=${LOCALE}.UTF-8\
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 repository and install the distribution.
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install --no-install-recommends --yes \
    ros-dev-tools \
    ros-${ROS_DISTRIBUTION}-ackermann-msgs \
    ros-${ROS_DISTRIBUTION}-desktop \
    ros-${ROS_DISTRIBUTION}-diagnostic-msgs \
    ros-${ROS_DISTRIBUTION}-rosbag2-storage-mcap \ 
    ros-${ROS_DISTRIBUTION}-vision-msgs && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install additional dependencies for debugging and documentation.
RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
    doxygen \
    gdb \
    graphviz \
    libeigen3-dev \
    libmodbus-dev \
    nlohmann-json3-dev \
    pipx \
    python3 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

################################################################################
# Add unprivileged user and set up profile scripts.                            #
################################################################################

RUN useradd --create-home --shell /bin/bash ${USER} && \
    echo "export LANG=${LOCALE}.UTF-8" >> /home/${USER}/.profile && \
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> /home/${USER}/.profile && \
    echo "source /home/${USER}/ROS/build/setup.bash  2> /dev/null" >> /home/${USER}/.profile && \
    echo "export ROS_LOG_DIR=\"/home/${USER}/ROS/log\"" >> /home/${USER}/.profile && \
    echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib" >> /home/${USER}/.profile

USER ${USER}

# Set the default shell to a Bash login shell
SHELL ["/bin/bash", "-l", "-c"]

# Install the poetry in the pipx virtual environemnt for the unprivileged user.
RUN pipx install poetry

################################################################################
# Set up the ROS workspace                                                     #
################################################################################

RUN mkdir -p ${HOME}/ROS/{build,install,log,src}

# Set the working directory to the ROS workspace
WORKDIR /home/${USER}/ROS

VOLUME ["/home/${USER}/ROS/build", "/home/${USER}/ROS/install"]

#

ARG SERIAL_ROS2_PATH="/home/${USER}/ROS/src/RoverRobotics-forks/serial-ros2"

RUN mkdir -p ${SERIAL_ROS2_PATH} && \
    git clone https://github.com/RoverRobotics-forks/serial-ros2.git ${SERIAL_ROS2_PATH}


################################################################################
# Set up container command                                                     #
################################################################################

CMD [ "/bin/bash", "-l", "-c" ]

################################################################################
# Define container labels                                                      #
################################################################################

LABEL org.opencontainers.image.source=https://github.com/aarhus-robotics/olav.git
LABEL org.opencontainers.image.revision=main
LABEL org.opencontainers.image.name=olav