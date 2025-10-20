<div align="center">

# ArduGazeboSim-Docker

[![Docker](https://img.shields.io/badge/docker-supported-blue.svg)](https://www.docker.com/)
[![VS Code](https://img.shields.io/badge/vscode-devcontainer-green.svg)](https://code.visualstudio.com/)
[![ROS](https://img.shields.io/badge/ros-noetic-34a853.svg)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/gazebo-11-orange.svg)](http://gazebosim.org/)
[![ArduPilot](https://img.shields.io/badge/ardupilot-SITL-yellow.svg)](https://ardupilot.org/)
[![MAVLink](https://img.shields.io/badge/mavlink-supported-red.svg)](https://mavlink.io/en/)
[![MAVROS](https://img.shields.io/badge/mavros-integration-blueviolet.svg)](http://wiki.ros.org/mavros)

🎥 **Demo Video**  
[![Watch the video](https://img.youtube.com/vi/CIGGLXief54/0.jpg)](https://youtu.be/CIGGLXief54)

🇬🇧 [English](#english) | 🇹🇷 [Türkçe](#türkçe)

</div>

---

<a name="english"></a>

**ArduGazeboSim-Docker** is a comprehensive Docker-based simulation environment for ArduPilot and Gazebo Classic 11, designed for drone development and testing. This project provides a complete containerized setup with ROS Noetic, allowing developers to work seamlessly across different machines without worrying about dependencies.

## Features

- ✅ **Containerized Environment**: Docker-based setup with all dependencies pre-installed
- ✅ **VS Code Integration**: Full development environment with Dev Containers
- ✅ **Multi-Drone Support**: Simulate multiple drones simultaneously
- ✅ **ROS Noetic Integration**: Complete ROS ecosystem for drone control
- ✅ **Gazebo Classic 11**: Realistic physics simulation
- ✅ **MAVProxy Support**: Direct connection to ArduPilot SITL
- ✅ **Cross-Platform**: Works on any system with Docker support
- ✅ **Persistent Storage**: Project files stored on host machine

## Prerequisites

- Docker and Docker Compose
- VS Code with Docker, Dev Containers and Remote - Containers extensions
- Sufficient disk space (~10GB for initial setup)

## Installation

### 1. Host Machine Preparation

#### 🟢 Ubuntu / Debian

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Docker and Docker Compose
sudo apt install -y docker.io docker-compose docker-buildx-plugin

# Enable and start Docker
sudo systemctl enable --now docker

# Add user to docker group (logout and login required)
sudo usermod -aG docker $USER

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
  && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Allow X11 forwarding for GUI applications
xhost +local:
```

---

#### 🔵 Arch Linux

```bash
# Update system
sudo pacman -Syu --noconfirm

# Install Docker and Docker Compose
sudo pacman -S --noconfirm docker docker-compose docker-buildx

# Enable and start Docker
sudo systemctl enable --now docker

# Add user to docker group (logout and login required)
sudo usermod -aG docker $USER

# Install NVIDIA Container Toolkit
yay -S --noconfirm nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Allow X11 forwarding for GUI applications
xhost +local:
```

---

#### 🟠 Fedora

```bash
# Update system
sudo dnf -y update

# Install Docker and Docker Compose
sudo dnf -y install docker docker-compose docker-buildx

# Enable and start Docker
sudo systemctl enable --now docker

# Add user to docker group (logout and login required)
sudo usermod -aG docker $USER

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo rpm --import - \
  && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/yum.repos.d/nvidia-container-toolkit.repo

sudo dnf -y clean expire-cache
sudo dnf -y install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Allow X11 forwarding for GUI applications
xhost +local:
```

---

### 2. Project Setup

```bash
# Create project directory
mkdir -p ~/ArduGazeboSim
cd ~/ArduGazeboSim
code .
```

### 3. Docker Configuration

Create a `Dockerfile` in the project root:

```dockerfile
FROM osrf/ros:noetic-desktop-full

# Install system dependencies
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
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir future lxml pymavlink MAVProxy osrf-pycommon empy

# Uncomment for GPU support
# apt-get update && apt-get install -y mesa-utils

# Create project directory
RUN mkdir -p /home/user/drone_project
WORKDIR /home/user/drone_project

# Set PATH for ArduPilot tools
ENV PATH="/home/user/drone_project/ardupilot:/home/user/drone_project/ardupilot/Tools/autotest:${PATH}"

CMD ["/bin/bash"]
```

### 4. Dev Container Configuration

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
    // Uncomment for GPU support
    //"--gpus=all",
    //"--device=/dev/dri:/dev/dri"
    "--net=host",
    "--env=DISPLAY=${env:DISPLAY}",
    "--env=QT_X11_NO_MITSHM=1",
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw",
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

### 5. Start the Container

In VS Code:
1. Locate the **`><` icon** in the **bottom-left corner** of the VS Code window.
2. Click it and select **“Reopen in Container”** from the menu.
3. Wait for the container to build and start (first time may take 10-15 minutes)

### 6. Install ROS and MAVROS

Instead of performing all installations manually, you can run the **setup\_simulation.sh** script prepared inside the Docker container. This will automatically install all dependencies.

#### Steps to Run:

1. Make the script executable inside the Docker container:

```bash
chmod +x setup_simulation.sh
```

2. Start the script:

```bash
./setup_simulation.sh
```

> **Note:** If you get errors like `PreArm: Main loop slow` or `Arm: COMPONENT_ARM_DISARM: FAILED`, run this in the same terminal as SITL:
>
> ```bash
> param set ARMING_CHECK 0
> ```
>
> This disables pre-arm checks for simulation.

#### Manual Setting

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

### 7. Install ArduPilot

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

### 8. Install ArduPilot Gazebo Plugin

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

## Usage

### Running the Simulation

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


## Tips and Recommendations

- **Persistent Environment:** All project files are stored on your host machine. The container can be stopped and restarted without losing your work.
- **Rebuilding:** If you modify the Dockerfile, you can **update the container** by clicking the **`><` icon** in the bottom-left corner of VS Code again and selecting **“Rebuild Container”**. VS Code will automatically rebuild the container with your changes.
- **GPU Support:** For GPU acceleration, add `"--gpus=all"` to `runArgs` in `devcontainer.json` and install NVIDIA Docker.

Happy flying! 🚁✨

---

<a name="türkçe"></a>

**ArduGazeboSim-Docker**, ArduPilot ve Gazebo Classic 11 için kapsamlı bir Docker tabanlı simülasyon ortamıdır ve drone geliştirme ve testleri için tasarlanmıştır. Bu proje, ROS Noetic ile tam bir konteynerleştirilmiş kurulum sağlayarak geliştiricilerin bağımlılıklar konusunda endişelenmeden farklı makinelerde sorunsuz çalışmasına olanak tanır.

## Özellikler

- ✅ **Konteynerleştirilmiş Ortam**: Tüm bağımlılıklar önceden yüklenmiş Docker tabanlı kurulum
- ✅ **VS Code Entegrasyonu**: Dev Containers ile tam geliştirme ortamı
- ✅ **Çoklu Drone Desteği**: Aynı anda birden fazla drone simülasyonu
- ✅ **ROS Noetic Entegrasyonu**: Drone kontrolü için eksiksiz ROS ekosistemi
- ✅ **Gazebo Classic 11**: Gerçekçi fizik simülasyonu
- ✅ **MAVProxy Desteği**: ArduPilot SITL'e doğrudan bağlantı
- ✅ **Çapraz Platform**: Docker desteği olan her sistemde çalışır
- ✅ **Kalıcı Depolama**: Proje dosyaları ana makinede saklanır

## Ön Gereksinimler

- Docker ve Docker Compose
- Docker, Dev Containers ve Remote - Containers uzantılarına sahip VS Code
- Yeterli disk alanı (ilk kurulum için ~10GB)

## Kurulum

### 1. Ana Makine Hazırlığı

#### 🟢 Ubuntu / Debian

```bash
# Sistemi güncelle
sudo apt update && sudo apt upgrade -y

# Docker ve Docker Compose kur
sudo apt install -y docker.io docker-compose docker-buildx-plugin

# Docker'ı etkinleştir ve başlat
sudo systemctl enable --now docker

# Kullanıcıyı docker grubuna ekle (çıkış yapıp tekrar giriş gereklidir)
sudo usermod -aG docker $USER

# NVIDIA Container Toolkit kur
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
  && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# GUI uygulamaları için X11 yönlendirmeye izin ver
xhost +local:
```

---

#### 🔵 Arch Linux

```bash
# Sistemi güncelle
sudo pacman -Syu --noconfirm

# Docker ve Docker Compose kur
sudo pacman -S --noconfirm docker docker-compose docker-buildx

# Docker'ı etkinleştir ve başlat
sudo systemctl enable --now docker

# Kullanıcıyı docker grubuna ekle (çıkış yapıp tekrar giriş gereklidir)
sudo usermod -aG docker $USER

# NVIDIA Container Toolkit kur
yay -S --noconfirm nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# GUI uygulamaları için X11 yönlendirmeye izin ver
xhost +local:
```

---

#### 🟠 Fedora

```bash
# Sistemi güncelle
sudo dnf -y update

# Docker ve Docker Compose kur
sudo dnf -y install docker docker-compose docker-buildx

# Docker'ı etkinleştir ve başlat
sudo systemctl enable --now docker

# Kullanıcıyı docker grubuna ekle (çıkış yapıp tekrar giriş gereklidir)
sudo usermod -aG docker $USER

# NVIDIA Container Toolkit kur
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo rpm --import - \
  && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/yum.repos.d/nvidia-container-toolkit.repo

sudo dnf -y clean expire-cache
sudo dnf -y install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# GUI uygulamaları için X11 yönlendirmeye izin ver
xhost +local:
```

---

### 2. Proje Kurulumu

```bash
# Proje dizini oluştur
mkdir -p ~/ArduGazeboSim
cd ~/ArduGazeboSim
code .
```

### 3. Docker Yapılandırması

Proje kök dizininde bir `Dockerfile` oluşturun:

```dockerfile
FROM osrf/ros:noetic-desktop-full

# Sistem bağımlılıklarını kur
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
    && rm -rf /var/lib/apt/lists/*

# Python paketlerini kur
RUN pip3 install --no-cache-dir future lxml pymavlink MAVProxy osrf-pycommon empy

# Ekran kartı için yorum satırını kaldırın
# apt-get update && apt-get install -y mesa-utils

# Proje dizini oluştur
RUN mkdir -p /home/user/drone_project
WORKDIR /home/user/drone_project

# ArduPilot araçları için PATH ayarla
ENV PATH="/home/user/drone_project/ardupilot:/home/user/drone_project/ardupilot/Tools/autotest:${PATH}"

CMD ["/bin/bash"]
```

### 4. Dev Container Yapılandırması

`.devcontainer/devcontainer.json` dosyası oluşturun:

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
    // Ekran kartı için yorum satırlarını kaldırın.
    //"--gpus=all",
    //"--device=/dev/dri:/dev/dri"
    "--net=host",
    "--env=DISPLAY=${env:DISPLAY}",
    "--env=QT_X11_NO_MITSHM=1",
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw",
  ]
}
```
### 📂 Proje Klasör Yapısı

```
ArduGazeboSim/
├── .devcontainer/                  # VS Code Dev Container ayarları
│   └── devcontainer.json           # Konteyner kurulumu için Dockerfile'a referans verir
│
├── Dockerfile                      # Geliştirme ortamı için ana Dockerfile
│
....
│   (proje dosyalarınız buraya gelir)
```

### 5. Konteyneri Başlatma

VS Code'da:
1. VS Code penceresinin **sol alt köşesindeki** **`><` simgesini** bulun.
2. Tıklayın ve menüden **"Reopen in Container"** seçeneğini seçin.
3. Konteynerin oluşturulup başlamasını bekleyin (ilk sefer 10-15 dakika sürebilir)

### 6. ROS ve MAVROS Kurulumu

Tüm kurulumları manuel olarak yapmak yerine, Docker konteyneri içinde hazırlanmış **setup_simulation.sh** betiğini çalıştırabilirsiniz. Bu, tüm bağımlılıkları otomatik olarak kuracaktır.

#### Çalıştırma Adımları:

1. Docker konteyneri içinde betiği çalıştırılabilir yapın:

```bash
chmod +x setup_simulation.sh
```

2. Betiği başlatın:

```bash
./setup_simulation.sh
```

> **Not:** `PreArm: Main loop slow` veya `Arm: COMPONENT_ARM_DISARM: FAILED` gibi hatalar alırsanız, SITL ile aynı terminalde şunu çalıştırın:
>
> ```bash
> param set ARMING_CHECK 0
> ```
>
> Bu, simülasyon için ön-arm kontrollerini devre dışı bırakır.

#### Manuel Ayar

Konteyner oluşturulduktan sonra, VS Code içinde (konteyner içinde) bir terminal açın ve aşağıdaki komutları adım adım çalıştırın.

```bash
# Catkin çalışma alanı oluştur
cd /home/user/drone_project
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin init

# MAVROS ve MAVLink kur
apt-get update && apt-get install -y software-properties-common && add-apt-repository universe
wstool init src
rosinstall_generator --rosdistro noetic --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator --rosdistro noetic mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# IQ Simulation paketini klonla
cd src
git clone https://github.com/Intelligent-Quads/iq_sim.git

# Gazebo model yolunu ayarla
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/user/drone_project/catkin_ws/src/iq_sim/models" >> ~/.bashrc

# Catkin çalışma alanını derle
cd /home/user/drone_project/catkin_ws
catkin build
echo "source /home/user/drone_project/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. ArduPilot Kurulumu

VS Code içindeki aynı terminalde, ArduPilot'u klonlamak ve derlemek için aşağıdaki komutları çalıştırın.

```bash
# ArduPilot'u klonla
cd /home/user/drone_project
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter

# ArduPilot araçları için PATH ayarla
echo "export PATH=\$PATH:/home/user/drone_project/ardupilot:/home/user/drone_project/ardupilot/Tools/autotest" >> ~/.bashrc
source ~/.bashrc
```

### 8. ArduPilot Gazebo Eklentisi Kurulumu

VS Code içindeki aynı terminalde, eklentiyi klonlamak ve derlemek için aşağıdaki komutları çalıştırın.

```bash
# Eklentiyi klonla ve derle
cd /home/user/drone_project/ardupilot
git clone https://github.com/khancyr/ardupilot_gazebo.git ardupilot_gazebo_classic
cd ardupilot_gazebo_classic
mkdir build && cd build
cmake ..
make -j$(nproc)
make install

# Ortam değişkenlerini ayarla
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/drone_project/ardupilot/ardupilot_gazebo_classic/models' >> ~/.bashrc
source ~/.bashrc
```

## Kullanım

### Simülasyonu Çalıştırma

1. **Terminal 1 - Gazebo ve ROS'u Başlat:**
   
   > VS Code içinde (konteyner içinde) **yeni bir terminal** açın ve simülasyon ortamını başlatın. Ardından şunu çalıştırın:
   
   ```bash
   source ~/.bashrc
   roslaunch iq_sim multi_drone.launch
   ```

4. **Terminal 2 - ArduCopter SITL Başlat:**

   > ArduPilot bağlantısı için VS Code içinde **başka bir yeni terminal** açın. Bu terminal drone SITL simülasyonunu yönetecektir:
   
   ```bash
   cd /home/user/drone_project/ardupilot
   sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map -I0
   ```

> **Not:** Simülasyon kurulumunu tamamladıktan sonra şunları yapabilirsiniz:
>
> * Konteyneri kullanarak VS Code içinde **kendi projelerinizi başlatın**.
> * Python betikleri çalıştırmak, ROS düğümlerini test etmek veya ek simülasyonlar başlatmak için konteyner içinde **yeni bir terminal** açın.


## İpuçları ve Öneriler

- **Kalıcı Ortam:** Tüm proje dosyaları ana makinenizde saklanır. Konteyner durdurulup yeniden başlatılabilir ve çalışmanız kaybolmaz.
- **Yeniden Derleme:** Dockerfile'ı değiştirirseniz, VS Code'un sol alt köşesindeki **`><` simgesine** tekrar tıklayarak ve **"Rebuild Container"** seçeneğini seçerek **konteyneri güncelleyebilirsiniz**. VS Code, değişikliklerinizle konteyneri otomatik olarak yeniden derleyecektir.
- **GPU Desteği:** GPU hızlandırma için `devcontainer.json` içindeki `runArgs` bölümüne `"--gpus=all"` ekleyin ve NVIDIA Docker kurun.

İyi uçuşlar! 🚁✨
