# Tobas (Raspbian)

## Setup

- Flash the OS to an SD card:
  [Raspberry Pi OS Lite](https://downloads.raspberrypi.com/raspios_lite_armhf/images/raspios_lite_armhf-2025-12-04/2025-12-04-raspios-trixie-armhf-lite.img.xz)

```bash
$ sudo mkdir -p /mnt/bootfs /mnt/rootfs
$ sudo mount /dev/sda1 /mnt/bootfs
$ sudo mount /dev/sda2 /mnt/rootfs
```

- Place each file under the root filesystem
- Configure the network and other settings with `tobas_bootmedia_config`
- Connect a display, USB keyboard, and LAN cable, then boot the Raspberry Pi
- Install the dependencies listed in `control`
- Follow `postinst`
- Power off once, then connect by SSH over Wi-Fi
- Apply the additional settings below

## Additional Settings

### Enable I2C and UART from raspi-config

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

- Enable CCACHE

```bash
$ sudo apt update && sudo apt install -y ccache
$ mkdir -p ~/.cache/ccache
$ echo "max_size = 5G" > ~/.cache/ccache/ccache.conf  # The default 5 GB is sufficient
$ ccache -s
```

- Add the following to `~/.bashrc`

```bash
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export CCACHE_DIR="$HOME/.cache/ccache/"
```

### Install ROS 2

```bash
$ ~/colcon_ws/src/tobas/tobas_dev_tools/scripts/install_ros2_raspbian
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
