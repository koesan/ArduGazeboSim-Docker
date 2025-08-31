#!/bin/bash
set -e

echo "===== Starting ArduGazeboSim setup ====="

# -------------------------------
# 1. Update apt and install dependencies
# -------------------------------
echo "Updating apt and installing dependencies..."
apt-get update

# List of system packages
packages=(
  git wget curl nano cmake build-essential
  python3-dev python3-pip python3-setuptools python3-wheel
  python3-matplotlib python3-numpy python3-pandas python3-scipy
  python3-sqlalchemy python3-pexpect python3-wstool
  python3-rosinstall-generator python3-catkin-lint python3-catkin-tools
  ros-noetic-geographic-msgs gazebo11 libgazebo11-dev
  software-properties-common
)

for pkg in "${packages[@]}"; do
    if dpkg -s "$pkg" >/dev/null 2>&1; then
        echo "$pkg is already installed."
    else
        echo "Installing $pkg..."
        apt-get install -y "$pkg"
    fi
done

# Python packages
python_packages=(future lxml pymavlink MAVProxy osrf-pycommon empy)
for pypkg in "${python_packages[@]}"; do
    if pip3 show "$pypkg" >/dev/null 2>&1; then
        echo "Python package $pypkg already installed."
    else
        echo "Installing Python package $pypkg..."
        pip3 install --no-cache-dir "$pypkg"
    fi
done

# -------------------------------
# 2. Setup Catkin Workspace
# -------------------------------
WORKSPACE_DIR="/home/user/drone_project/catkin_ws"
SRC_DIR="$WORKSPACE_DIR/src"
mkdir -p "$SRC_DIR"
cd "$WORKSPACE_DIR"

if [ ! -f "$WORKSPACE_DIR/.catkin_workspace" ]; then
    echo "Initializing catkin workspace..."
    source /opt/ros/noetic/setup.bash
    catkin init
else
    echo "Catkin workspace already initialized."
fi

# -------------------------------
# 3. Install MAVROS and MAVLink
# -------------------------------
cd "$SRC_DIR"
if [ ! -d "$SRC_DIR/mavros" ]; then
    echo "Setting up MAVROS and MAVLink..."
    wstool init . || echo "wstool already initialized"
    rosinstall_generator --rosdistro noetic --upstream mavros | tee /tmp/mavros.rosinstall
    rosinstall_generator --rosdistro noetic mavlink | tee -a /tmp/mavros.rosinstall
    wstool merge -t . /tmp/mavros.rosinstall
    wstool update -t .
else
    echo "MAVROS already cloned."
fi

echo "Installing ROS dependencies..."
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro noetic -y

# Clone IQ Simulation package
if [ ! -d "$SRC_DIR/iq_sim" ]; then
    git clone https://github.com/Intelligent-Quads/iq_sim.git
else
    echo "IQ Simulation package already cloned."
fi

# Set Gazebo model path
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$SRC_DIR/iq_sim/models" >> ~/.bashrc

# Build catkin workspace
cd "$WORKSPACE_DIR"
catkin build
echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# -------------------------------
# 4. Clone and build ArduPilot
# -------------------------------
DRONE_DIR="/home/user/drone_project/ardupilot"
mkdir -p "$DRONE_DIR"
cd /home/user/drone_project

if [ ! -d "$DRONE_DIR/.git" ]; then
    echo "Cloning ArduPilot..."
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
else
    echo "ArduPilot already cloned."
fi

cd "$DRONE_DIR"
./waf configure --board sitl
./waf copter

# Set PATH for ArduPilot tools
echo "export PATH=\$PATH:$DRONE_DIR:$DRONE_DIR/Tools/autotest" >> ~/.bashrc
source ~/.bashrc

# -------------------------------
# 5. Clone and build ArduPilot Gazebo plugin
# -------------------------------
PLUGIN_DIR="$DRONE_DIR/ardupilot_gazebo_classic"
cd "$DRONE_DIR"
if [ ! -d "$PLUGIN_DIR" ]; then
    git clone https://github.com/khancyr/ardupilot_gazebo.git ardupilot_gazebo_classic
fi

cd "$PLUGIN_DIR"
mkdir -p build && cd build
cmake ..
make -j$(nproc)
make install

# Set environment variables for Gazebo
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib' >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$PLUGIN_DIR/models" >> ~/.bashrc
source ~/.bashrc

# -------------------------------
# 6. AMD GPU Check
# -------------------------------
if [ -c /dev/dri/renderD128 ]; then
    echo "AMD GPU detected at /dev/dri/renderD128"
else
    echo "Warning: AMD GPU not detected. Simulation may run on CPU (llvmpipe)."
fi

echo "===== ArduGazeboSim setup completed successfully! ====="
