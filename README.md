

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
- VS Code with Docker and Remote - Containers extensions
- Sufficient disk space (~10GB for initial setup)

## Installation / Kurulum

### 1. Host Machine Preparation / Ana Makine Hazırlığı

```bash
# Install Docker and Docker Compose
sudo apt update
sudo apt install -y docker.io docker-compose
sudo systemctl enable --now docker

# Add user to docker group (logout and login after this)
sudo usermod -aG docker $USER

# Install VS Code extensions
# Install: Docker, Remote - Containers

# Allow X11 forwarding for GUI applications
xhost +local:
```

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

### 5. Start the Container / Konteyneri Başlatma

In VS Code:
1. Open the command palette (Ctrl+Shift+P)
2. Select "Remote-Containers: Reopen in Container"
3. Wait for the container to build and start (first time may take 10-20 minutes)

### 6. Install ROS and MAVROS / ROS ve MAVROS Kurulumu

Inside the container terminal:

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

Inside the container terminal:

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

Inside the container terminal:

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
   ```bash
   source ~/.bashrc
   roslaunch iq_sim multi_drone.launch
   ```

2. **Terminal 2 - Start ArduCopter SITL:**
   ```bash
   cd /home/user/drone_project/ardupilot
   sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map -I0
   ```

### Customization / Özelleştirme

- **Change Models:** Modify `.sdf` files in:
  - `~/drone_project/catkin_ws/src/iq_sim/models`
  - `~/drone_project/ardupilot/ardupilot_gazebo_classic/models`

- **Add Camera Sensors:** Add to drone model `.sdf`:
  ```xml
  <sensor type="camera" name="camera_sensor">
    <pose>0 0 0 0 0 0</pose>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
  </sensor>
  ```

- **Network Configuration:** Ports are forwarded via `--net=host`. For custom ports, modify `forwardPorts` in `devcontainer.json`.

## Tips and Recommendations / İpuçları ve Öneriler

- **Persistent Environment:** All project files are stored on your host machine. The container can be stopped and restarted without losing your work.
- **Performance:** For better performance, close unnecessary applications when running simulations.
- **Multiple Drones:** Use the `-I` flag in `sim_vehicle.py` with different numbers for additional drones (e.g., `-I1`, `-I2`).
- **Rebuilding:** If you modify the Dockerfile, VS Code will automatically rebuild the container when you reopen it.
- **GPU Support:** For GPU acceleration, add `"--gpus=all"` to `runArgs` in `devcontainer.json` and install NVIDIA Docker.

## Project Structure / Proje Yapısı

```
ArduGazeboSim/
├── .devcontainer/
│   └── devcontainer.json
├── Dockerfile
├── ardupilot/           # ArduPilot source
├── catkin_ws/           # ROS workspace
└── README.md
```

## Contributing / Katkıda Bulunma

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License / Lisans

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments / Teşekkür

- [ArduPilot](https://ardupilot.org/) for the amazing autopilot software
- [ROS](https://www.ros.org/) for the robotics framework
- [Gazebo](https://gazebosim.org/) for the simulation environment
- [Intelligent-Quads](https://github.com/Intelligent-Quads) for the IQ Sim package
- [Docker](https://www.docker.com/) for containerization technology
- [VS Code](https://code.visualstudio.com/) for the development environment

---

**English:** For questions or support, please open an issue on GitHub.

**Türkçe:** Sorularınız veya destek için lütfen GitHub'da bir issue oluşturun.

Happy flying! / İyi uçuşlar! 🚁✨
