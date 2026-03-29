FROM osrf/ros:noetic-desktop-full

# NVIDIA Ekran Kartı Grafik Yetkilerini Aç (Ağır Yük İçin)
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,utility

# Install system dependencies
# EKLENEN KRİTİK PAKETLER: libgl1-mesa-glx libgl1-mesa-dri (AMD Görüntü Köprüsü İçin)
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git wget curl nano cmake build-essential \
    python3-dev python3-pip python3-setuptools python3-wheel \
    python3-matplotlib python3-numpy python3-pandas python3-scipy \
    python3-sqlalchemy python3-pexpect python3-wstool \
    python3-rosinstall-generator python3-catkin-lint python3-catkin-tools \
    ros-noetic-geographic-msgs \
    gazebo11 libgazebo11-dev \
    python3-wxgtk4.0 \
    mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir future lxml pymavlink MAVProxy osrf-pycommon empy

# Create project directory
RUN mkdir -p /home/user/drone_project
WORKDIR /home/user/drone_project

# Set PATH for ArduPilot tools
ENV PATH="/home/user/drone_project/ardupilot:/home/user/drone_project/ardupilot/Tools/autotest:${PATH}"

CMD ["/bin/bash"]
