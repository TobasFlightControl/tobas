# Troubleshooting

This section summarizes known issues and their solutions.

## Gazebo is running slowly

---

### 1. The display server may not be set to X11

Open `Settings / System / About / System Details`
and check that `Windowing System` is set to X11.

If it is not X11—for example, if it shows Wayland—
select your username on the Ubuntu login screen,
then use the gear icon that appears in the bottom-right corner to select the display server as follows.

- If `Ubuntu` or `Ubuntu on Wayland` is available → `Ubuntu`
- If `Ubuntu on Xorg` or `Ubuntu` is available → `Ubuntu on Xorg`

### 2. The GPU driver may not be installed

Open `Settings / System / About / System Details`
and check that the correct GPU is shown in `Graphics`.

If the GPU shown does not match your PC's specifications, the driver may not be working correctly.
Install the appropriate driver for your GPU.

![troubleshooting/system_details](../../assets/troubleshooting/system_details.png)

## ROS communication between the FC and PC is not working

---

### 1. The firewall may be blocking UDP

ROS 2 uses UDP for network communication, but the firewall may not allow it.
Check the firewall status with the following command.
If the list of allowed ports does not include UDP ports in the 7400 range, this may be the cause.

```bash
$ sudo ufw status
```

Ideally, you should allow only the ports in use,
but for now, disabling UFW and rebooting should restore communication.

```bash
$ sudo ufw disable
$ sudo reboot
```

## The FC is not working after creating and flashing user code

---

A runtime error may have occurred.
Log in to the Raspberry Pi via SSH and check the console output with `journalctl` for possible clues.
Use the arrow keys to navigate and press `Q` to exit.

```bash
$ ssh pi@${hostname}.local  # or pi@${ip_address}
$ journalctl -u tobas_real_realtime.service -e  # or tobas_real_interface.service
```
