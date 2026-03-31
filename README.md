# Tobas

Tobas is a next-generation, model-based flight controller created to meet the demands of the rapidly expanding drone market,
where aircraft are becoming larger and increasingly specialized.
Unlike traditional controllers, Tobas takes each airframe’s unique physical properties into account when designing its control system,
enabling precise flight performance even for unconventional frames not supported by conventional solutions.

## Documentation

[Tobas User Guide](https://tobas-wiki-ja.readthedocs.io/ja/stable/)

## Supported Motor Controller Firmware

- v1.1

## Setup from source

### PC (Ubuntu 24.04 LTS)

1. [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

   ```bash
   # Set locale
   $ sudo apt update
   $ sudo apt install -y locales
   $ sudo locale-gen en_US en_US.UTF-8
   $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   $ export LANG=en_US.UTF-8

   # Enable required repositories
   $ sudo apt install -y software-properties-common
   $ yes "" | sudo add-apt-repository universe
   $ sudo apt update
   $ sudo apt install -y curl
   $ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
   $ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
   $ sudo dpkg -i /tmp/ros2-apt-source.deb
   $ sudo apt update

   # Install development tools (optional)
   $ sudo apt install -y ros-dev-tools
   $ sudo apt install -y python3-colcon-common-extensions
   $ sudo apt install -y python3-colcon-clean

   # Install ROS 2
   $ sudo apt install -y ros-jazzy-desktop

   # Initialize rosdep
   $ sudo rosdep init
   $ rosdep update
   ```

2. Create colcon workspace

   ```bash
   $ mkdir -p ~/colcon_ws/src
   $ cd ~/colcon_ws
   ```

3. Set up your system automatically each time a new shell is opened

   ```bash
   $ echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
   $ echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   $ echo "source ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc
   $ exec bash
   ```

4. Clone Tobas

   ```bash
   $ cd ~/colcon_ws/src
   $ git clone git@github.com:TobasFlightControl/tobas.git --recursive -b jazzy # or "jazzy-dev" if you contribute to Tobas
   ```

5. Install dependencies

   ```bash
   $ rosdep install --from-paths ~/colcon_ws/src/tobas -yi
   $ sudo apt install -y libgit2-dev
   ```

6. Build

   ```bash
   $ cd ~/colcon_ws
   $ colcon build --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to tobas
   ```

7. Add the user to the `dialout` group (required to read S.BUS over USB during SITL)

   ```bash
   $ sudo usermod -aG dialout $USER
   ```

8. Restart the PC to finish the installation

   ```bash
   $ sudo reboot
   ```

### FC (Debian Trixie)

1. Flash the Tobas image to the SD card and complete the initial setup
   - [Installation | Tobas Doc](https://tobas-wiki-ja.readthedocs.io/ja/stable/installation/)
   - [Boot Device Configuration | Tobas Doc](https://tobas-wiki-ja.readthedocs.io/ja/stable/bootmedia_config/)

2. Transfer the latest source code to the FC

   ```bash
   $ .../tobas/tobas_dev_tools/scripts/tobas_sync <host>  # e.g. tobas.local, 192.168.3.7
   ```

3. Log in to the FC

   ```bash
   $ ssh pi@<host>
   ```

4. Build and install

   ```bash
   pi@<hostname> $ /home/pi/colcon_ws/src/tobas/tobas_dev_tools/scripts/tobas_deploy_upto tobas
   ```

## For Contributors

See [Contributing to Tobas](./CONTRIBUTING.md).
