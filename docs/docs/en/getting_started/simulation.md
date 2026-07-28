# Simulation

## Starting and Stopping the Simulation

---

Launch Tobas GCS, load `tobas_f450.TBS`, and open `Simulation` from the toolbar.

![simulation_settings](../../assets/simulation/simulation_settings.png)

Click `Start` to build the project and start the simulation.
The first launch may take some time because the environment data must be downloaded.
Although this depends on the environment complexity and network conditions, the simulation usually starts within 10 minutes.

![launch_gazebo](../../assets/simulation/launch_gazebo.png)

You can configure environmental settings such as wind speed from `Dynamic Configurations` and send commands to the aircraft from `Commanders`.
Mission planning and parameter tuning introduced in [Flight Test](./flight_test.md) can also be performed in the same way as with an actual aircraft.

![send_command](../../assets/simulation/send_command.png)

Click `Terminate` to stop the simulation.

## Control via an RC Transmitter

---

You can control the aircraft in the simulation using an RC transmitter.
Because the RC calibration results stored on the PC are used,
complete the calibration on your PC before proceeding with the following steps.

### Preparing a USB-to-Serial Converter

Prepare a USB-to-serial converter such as the
<a href=https://akizukidenshi.com/catalog/g/g108461/ target="_blank">FTDI FT234X</a>
and configure it to invert High and Low.

### Adding the User to the `dialout` Group

To access USB serial devices, add the current user to the `dialout` group using the following command.

```bash
sudo usermod -aG dialout "${USER}"
```

Restart the PC after running the command to apply the change.
This is required only the first time and does not need to be repeated.

### Steps Required Each Time You Run the Simulation

1. Connect the USB-to-serial converter to the PC and RC receiver.
1. Select the device you are using in the `S.BUS/Device` field under `Simulation Settings`.
1. Start the simulation. If the RC transmitter signals appear in `Control System`, the setup is successful. You can then control the aircraft in the same way as an actual aircraft.

![rc_teleop_setting](../../assets/simulation/rc_teleop_setting.png)

## Control via ROS

---

All communication between the components of the flight controller is handled through ROS,
allowing users to control the aircraft from their own programs.
For details, see [User Code (Python)](../for_developers/user_code_py.md) and [User Code (C++)](../for_developers/user_code_cpp.md).
