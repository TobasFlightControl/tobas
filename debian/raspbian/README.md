# Tobas (Raspbian)

## Setup

### Prepare the SD card with Raspberry Pi Imager

Use a current [Raspberry Pi Imager 2.x](https://www.raspberrypi.com/software/) to configure Wi-Fi and SSH before the first boot.

1. Check the Imager version in its About dialog. Imager 1.x cannot apply the customisation used by newer Trixie images.
2. Select the Raspberry Pi model, choose **Raspberry Pi OS Lite (Trixie)** from Imager's official OS list, and select the SD card.
3. Complete OS customisation before writing the image:

| Setting       | Value                                                                              |
| ------------- | ---------------------------------------------------------------------------------- |
| Hostname      | A unique name, such as `tobas`                                                     |
| User          | Username `pi` and a password; Tobas uses `/home/pi`                                |
| Localisation  | The appropriate time zone, keyboard layout, and Wi-Fi country                      |
| Wi-Fi         | The SSID and password for the network used by the PC; enable Hidden SSID if needed |
| Remote access | Enable SSH and configure password authentication or the PC's SSH public key        |

Write the image, wait for verification to finish, and eject the SD card.

### First boot and SSH connection

Insert the SD card into the Raspberry Pi and connect only its power supply.
No HDMI display, USB keyboard, or LAN cable is required.
Allow a few minutes for the initial configuration and Wi-Fi connection to complete.

From a PC on the same network, connect using the hostname configured in Imager:

```bash
$ ssh pi@<hostname>.local
```

### Set locale

https://zenn.dev/thorie/articles/548emb-install-ros2-offical-ver-on-raspberry-pi-os

### Apply Tobas settings over SSH

Complete the first boot before placing the Tobas files on the device: the overlay includes
`/etc/cloud/cloud-init.disabled`, which can prevent Imager's first-boot customisation from running.

1. Transfer the Tobas files from the PC over SSH and place each file under the corresponding root filesystem path.
   When applying `boot/firmware/cmdline.txt`, retain the SD card's actual `root=PARTUUID=...` value
   and match `cfg80211.ieee80211_regdom` to the Wi-Fi country selected in Imager.
2. Install the dependencies listed in [control](DEBIAN/control).
3. Prepare Wi-Fi for the Tobas network configuration.
4. Follow [postinst](DEBIAN/postinst), then apply the additional settings below through the SSH session.
5. Reboot with `sudo reboot`, wait for Wi-Fi to reconnect, and log in again with `ssh pi@<hostname>.local`.

## Additional Settings

### Enable I2C and UART from raspi-config

Run `sudo raspi-config` in the SSH session.

- I2C: Interface Options -> I2C
- UART: Interface Options -> Serial Port -> Shell: No, Hardware: Yes

### Disable the Network Installation UI

Open the configuration file with `rpi-eeprom-config`:

```
$ sudo EDITOR=emacs rpi-eeprom-config --edit
```

Edit the `NET_INSTALL_AT_POWER_ON` line:

```
NET_INSTALL_AT_POWER_ON=0
```

### CCACHE

Enable CCACHE:

```bash
$ sudo apt update && sudo apt install -y ccache
$ mkdir -p ~/.cache/ccache
$ echo "max_size = 5G" > ~/.cache/ccache/ccache.conf  # The default 5 GB is sufficient
$ ccache -s
```

Add the following to `~/.bashrc`:

```bash
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export CCACHE_DIR="$HOME/.cache/ccache/"
```

### Install ROS 2

Run the installer:

```bash
$ ~/colcon_ws/src/tobas/tools/setup/install_ros2_raspbian
```

## Notes

### `/boot/firmware/cmdline.txt`

- `console=tty1`:
  Disable the serial console
- `root=PARTUUID=xxxxxxxx-02`:
  Specify the root filesystem partition (must match the contents of `/etc/fstab`)
- `rcu_nocbs=1-3`:
  Avoid RCU (Read-Copy-Update) to be executed on CPU1-3
- `nohz_full=1-3`:
  Avoid periodical timer interruption to be executed on CPU1-3
- `isolcpus=domain,managed_irq,1-3`:
  Isolate CPU1-3
- `irqaffinity=0`:
  Set the default IRQ affinity to CPU0
- `resize`:
  Resize (Expand) the rootfs partition (not filesystem)
