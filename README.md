

# ArduGazeboSim-Docker

[![Docker](https://img.shields.io/badge/docker-supported-blue.svg)](https://www.docker.com/)
[![VS Code](https://img.shields.io/badge/vscode-devcontainer-green.svg)](https://code.visualstudio.com/)
[![ROS](https://img.shields.io/badge/ros-noetic-34a853.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**ArduGazeboSim-Docker** is a comprehensive Docker-based simulation environment for ArduPilot and Gazebo Classic 11, designed for drone development and testing. This project provides a complete containerized setup with ROS Noetic, allowing developers to work seamlessly across different machines without worrying about dependencies.

## Features / Özellikler

- ✅ **Containerized Environment**: Docker-based setup with all dependencies pre-installed
- ✅ **VS Code Integration**: Full development environment with Dev Containers
- ✅ **Multi-Drone Support**: Simulate multiple drones simultaneously
- ✅ **ROS Noetic Integration**: Complete ROS ecosystem for drone control
- ✅ **Gazebo Classic 11**: Realistic physics simulation
- ✅ **MAVProxy Support**: Direct connection to ArduPilot SITL
- ✅ **Cross-Platform**: Works on any system with Docker support
- ✅ **Persistent Storage**: Project files stored on host machine

## Prerequisites / Ön Gereksinimler

- Docker and Docker Compose
- VS Code with Docker, Dev Containers and Remote - Containers extensions
- Sufficient disk space (~10GB for initial setup)

## Installation / Kurulum

### 1. Host Machine Preparation / Ana Makine Hazırlığı

#### 🟢 Ubuntu / Debian

```bash
# Install Docker and Docker Compose
sudo apt update
sudo apt install -y docker.io docker-compose
sudo systemctl enable --now docker

# Add user to docker group (logout and login after this)
sudo usermod -aG docker $USER

# Install VS Code extensions
# Install: Docker,Dev Containers and Remote - Containers

# Allow X11 forwarding for GUI applications
xhost +local:

# Allow Wayland socket access for GUI applications
xhost +SI:localuser:$USER
```

---

#### 🔵 Arch Linux

```bash
# Install Docker and Docker Compose
sudo pacman -Syu --noconfirm
sudo pacman -S --noconfirm docker docker-compose

# Enable and start Docker service
sudo systemctl enable --now docker

# Add user to docker group (logout and login after this)
sudo usermod -aG docker $USER

# Install VS Code extensions
# Install: Docker,Dev Containers and Remote - Containers

# Allow X11 forwarding for GUI applications
xhost +local:

# Allow Wayland socket access for GUI applications
xhost +SI:localuser:$USER
```

---

#### 🟠 Fedora

```bash
# Install Docker and Docker Compose
sudo dnf -y update
sudo dnf -y install docker docker-compose

# Enable and start Docker service
sudo systemctl enable --now docker

# Add user to docker group (logout and login after this)
sudo usermod -aG docker $USER

# Install VS Code extensions
# Install: Docker,Dev Containers and Remote - Containers

# Allow X11 forwarding for GUI applications
xhost +local:

# Allow Wayland socket access for GUI applications
xhost +SI:localuser:$USER
```

---

### 2. Project Setup / Proje Kurulumu

```bash
# Create project directory
mkdir -p ~/ArduGazeboSim
cd ~/ArduGazeboSim
code .
```

### 3. Docker Configuration / Docker Yapılandırması

Create a `Dockerfile` in the project root:

```dockerfile
FROM osrf/ros:noetic-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git wget curl nano cmake build-essential \
    python3-dev python3-pip python3-setuptools python3-wheel \
    python3-matplotlib python3-numpy python3-pandas python3-scipy \
    python3-sqlalchemy python3-pexpect python3-wstool \
    python3-rosinstall-generator python3-catkin-lint python3-catkin-tools \
    ros-noetic-geographic-msgs \
    gazebo11 libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir future lxml pymavlink MAVProxy osrf-pycommon empy

# Create project directory
RUN mkdir -p /home/user/drone_project
WORKDIR /home/user/drone_project

# Set PATH for ArduPilot tools
ENV PATH="/home/user/drone_project/ardupilot:/home/user/drone_project/ardupilot/Tools/autotest:${PATH}"

CMD ["/bin/bash"]
```

### 4. Dev Container Configuration / Dev Container Yapılandırması

Create `.devcontainer/devcontainer.json`:

```json
{
  "name": "ArduGazeboSim Environment",
  "dockerFile": "../Dockerfile",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-azuretools.vscode-docker",
        "ms-vscode-remote.remote-containers"
      ]
    }
  },
  "forwardPorts": [8100, 8200, 8300],
  "mounts": [
    "source=${localWorkspaceFolder},target=/home/user/drone_project,type=bind"
  ],
  "runArgs": [
    "--net=host",
    "--env=DISPLAY=${env:DISPLAY}",
    "--env=QT_X11_NO_MITSHM=1",
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
  ]
}
```
### 📂 Project Folder Structure

```
ArduGazeboSim/
├── .devcontainer/                  # VS Code Dev Container settings
│   └── devcontainer.json           # References the Dockerfile for container setup
│
├── Dockerfile                      # Main Dockerfile for the development environment
│
....
│   (your project files go here)
```

### 5. Start the Container / Konteyneri Başlatma

In VS Code:
1. Locate the **`><` icon** in the **bottom-left corner** of the VS Code window.
2. Click it and select **“Reopen in Container”** from the menu.
3. Wait for the container to build and start (first time may take 10-15 minutes)

### 6. Install ROS and MAVROS / ROS ve MAVROS Kurulumu

After the container is built, open a terminal inside VS Code (inside the container) and run the following commands step by step.

```bash
# Create catkin workspace
cd /home/user/drone_project
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin init

# Install MAVROS and MAVLink
apt-get update && apt-get install -y software-properties-common && add-apt-repository universe
wstool init src
rosinstall_generator --rosdistro noetic --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator --rosdistro noetic mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Clone IQ Simulation package
cd src
git clone https://github.com/Intelligent-Quads/iq_sim.git

# Set Gazebo model path
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/user/drone_project/catkin_ws/src/iq_sim/models" >> ~/.bashrc

# Build catkin workspace
cd /home/user/drone_project/catkin_ws
catkin build
echo "source /home/user/drone_project/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. Install ArduPilot / ArduPilot Kurulumu

In the same terminal inside VS Code, run the following commands to clone and build ArduPilot.

```bash
# Clone ArduPilot
cd /home/user/drone_project
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter

# Set PATH for ArduPilot tools
echo "export PATH=\$PATH:/home/user/drone_project/ardupilot:/home/user/drone_project/ardupilot/Tools/autotest" >> ~/.bashrc
source ~/.bashrc
```

### 8. Install ArduPilot Gazebo Plugin / ArduPilot Gazebo Eklentisi Kurulumu

In the same terminal inside VS Code, run the following commands to clone and build ArduPilot.

```bash
# Clone and build the plugin
cd /home/user/drone_project/ardupilot
git clone https://github.com/khancyr/ardupilot_gazebo.git ardupilot_gazebo_classic
cd ardupilot_gazebo_classic
mkdir build && cd build
cmake ..
make -j$(nproc)
make install

# Set environment variables
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/drone_project/ardupilot/ardupilot_gazebo_classic/models' >> ~/.bashrc
source ~/.bashrc
```

## Usage / Kullanım

### Running the Simulation / Simülasyonu Çalıştırma

1. **Terminal 1 - Start Gazebo and ROS:**
   
   > Open a **new terminal inside VS Code** (inside the container) to start the simulation environment. Then run:
   
   ```bash
   source ~/.bashrc
   roslaunch iq_sim multi_drone.launch
   ```

4. **Terminal 2 - Start ArduCopter SITL:**

   > Open **another new terminal inside VS Code** for ArduPilot connection. This terminal will handle drone SITL simulation:
   
   ```bash
   cd /home/user/drone_project/ardupilot
   sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map -I0
   ```

> **Note:** After completing the simulation setup, you can:
>
> * Use the container to **start your own projects** inside VS Code.
> * Open a **new terminal** inside the container to run Python scripts, test ROS nodes, or start additional simulations.


## Tips and Recommendations / İpuçları ve Öneriler

- **Persistent Environment:** All project files are stored on your host machine. The container can be stopped and restarted without losing your work.
- **Rebuilding:** If you modify the Dockerfile, you can **update the container** by clicking the **`><` icon** in the bottom-left corner of VS Code again and selecting **“Rebuild Container”**. VS Code will automatically rebuild the container with your changes.
- **GPU Support:** For GPU acceleration, add `"--gpus=all"` to `runArgs` in `devcontainer.json` and install NVIDIA Docker.

Happy flying! / İyi uçuşlar! 🚁✨
