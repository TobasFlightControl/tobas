# Simulation

## Startup and Shutdown Procedure

---

Launch `TobasGCS`, load `tobas_f450.TBS`, and open `Simulation` from the tool button.

![simulation_settings](../../assets/simulation/simulation_settings.png)

Click `Start` to build the project and then start the simulation.
The first launch may take some time because environment data needs to be downloaded.
Depending on the complexity of the environment and your network conditions, it will usually start within 10 minutes at most.

![launch_gazebo](../../assets/simulation/launch_gazebo.png)

You can configure environmental settings such as wind speed from `Dynamic Configurations`, and send commands to the vehicle from `Commanders`.
You can also perform mission planning and parameter tuning introduced in [Flight Test](./flight_test.md), just as you would on the real vehicle.

![send_command](../../assets/simulation/send_command.png)

Click `Terminate` to end the simulation.

## Operation with an RC Transmitter

---

You can operate the vehicle in the simulation using an RC transmitter.
Since the RC calibration results saved on the PC are used,
complete the calibration on your PC before proceeding with the steps below.

1. Prepare a USB-to-serial conversion module such as
   <a href=https://akizukidenshi.com/catalog/g/g108461/ target="_blank">FTDI FT234X</a>.
1. Configure the USB-to-serial conversion module to invert High and Low.
1. Connect the USB-to-serial conversion module to the PC and the RC receiver.
1. Select the device you are using from the `S.BUS/Device` field in `Simulation Settings`.
1. If the transmitter signal appears in `Control System`, the setup is successful. You can then operate it just as you would on the real vehicle.

![rc_teleop_setting](../../assets/simulation/rc_teleop_setting.png)

## Operation via ROS

---

Since communication between all components that make up the flight controller is handled through ROS (ROS 2 Jazzy),
users can control the vehicle from their own programs.
For details, see [User Code (Python)](../for_developers/user_code_py.md) and [User Code (C++)](../for_developers/user_code_cpp.md).
