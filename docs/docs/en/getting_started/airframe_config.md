# Airframe Configuration

<!-- ゲームの広告と同じで，全てを理解することよりもとりあえず何も考えずに簡単に動かせることが大事． -->
<!-- 後々必要になる面倒な作業は隠して面白いところを見せる． -->

Use Tobas Setup Assistant to configure the airframe.
Tobas Setup Assistant is a GUI for creating the project folder required to fly a drone using Tobas.
The project folder contains all the information required to fly the drone, including the aircraft's mass properties, the propeller's aerodynamic properties, and the motor's electrical properties.
To use Tobas Setup Assistant, you need a Universal Aircraft Description Format (UADF) file that describes your aircraft.
For details about UADF, see [What is UADF](../additional_information/what_is_uadf.md).

## Preparation

---

This tutorial uses the DJI F450, a typical quadcopter.
The components are as follows:

- Flight controller: <a href=https://tobas.jp/product target="_blank">Tobas FC201</a>
- Power module: <a href=https://holybro.com/products/pm02d-power-module target="_blank">Holybro PM02D</a>
- Frame: <a href=https://www.hawks-work.com/products/f450-drone-frame-450mm-wheelbase-quadcopter-frame-kit-with-landing-skid-gear target="_blank">DJI F450 Frame</a>
- Motor: <a href=https://www.hawks-work.com/products/a2212-brushless-motor-920kv-for-multirotor-drone-and-others target="_blank">A2212 920KV</a> (CW x 2, CCW x 2)
- Propeller: <a href=https://www.hawks-work.com/products/9450-propellers-self-tightening-cw-ccw-propeller-for-multirotor-f450 target="_blank">DJI 9450</a> (CW x 2, CCW x 2)
- ESC: <a href="https://www.fly-color.net/index.php?c=category&id=234" target="_blank">Flycolor Raptor5 35A</a> x 4
- Battery: <a href=https://hrb-power.com/products/11-1v-5000mah-6000mah-50c-trx target="_blank">HRB 3S 5000mAh 50C</a>
- GNSS antenna: <a href=https://www.topgnss.store/en-jp/products/top901-gnss-l1-l2-l5-unmanned-vehicle-high-precision-drone-measurement-full-frequency-rtk-antenna-new-small-high-gain-helical target="_blank">TOPGNSS TOP901</a>
- RC receiver: <a href=https://www.rc.futaba.co.jp/products/detail/I00000021 target="_blank">Futaba R2000SBM</a>

You need to create a UADF file for the aircraft, but this tutorial uses a file that has already been prepared.

## Launch

---

Run the following command in a terminal to launch Tobas Setup Assistant.

```bash
$ ros2 launch tobas_setup_assistant setup_assistant.launch.py
```

![start](../../assets/airframe_config/start.png)

## Loading the UADF

---

Click `New`, select `/opt/ros/jazzy/share/tobas_description/urdf/f450.uadf` in the file dialog, and click `Open`.
The aircraft will appear in the model view, and each configuration page will be enabled.

![load](../../assets/airframe_config/load.png)

## Propulsion System

---

Configure the propulsion system.
Because this aircraft is electrically powered, leave `Electric Propulsion System` checked.

### Battery

Configure the battery.
Check the battery specifications and enter the appropriate value for each field.

![battery](../../assets/airframe_config/propulsion/battery.png)

### Propulsion Units

Configure each propulsion unit.

First, configure the `propeller_0` link.
Check the specifications of each component and enter the appropriate values for `ESC`, `Motor`, and `Propeller`.

!!! tip

    If you are unsure which link name corresponds to a propeller on the aircraft,
    click the link name under `Frame Tree` in the upper-left corner to highlight it in the model view.

![propulsion/esc](../../assets/airframe_config/propulsion/esc.png)

![propulsion/motor](../../assets/airframe_config/propulsion/motor.png)

![propulsion/propeller](../../assets/airframe_config/propulsion/propeller.png)

Configure the propeller's aerodynamic properties under `Aerodynamics`.
You can choose from several configuration methods. Because a model is already available for the DJI 9450 propeller used in this tutorial, use that model.
Select `Select Propeller Model` from the first list, then select `dji_9450` from the list below it.

![propulsion/aerodynamics](../../assets/airframe_config/propulsion/aerodynamics.png)

Because all four propulsion units on this aircraft are identical, click `Copy To All` to copy the `propeller_0` settings to the other three units.
Confirm that the `propeller_0` settings have also been applied to the other tabs.

## Hardware Interface

---

Configure the hardware connections.
Confirm that `Tobas FC2xx` is selected,
then set the appropriate DShot channel for each of the four propulsion units.

![hardware_interface](../../assets/airframe_config/hardware_interface.png)

!!! note

    If you do not specify an interface here, such as when using a CAN-ESC,
    the hardware will not operate as-is.
    You must create a custom ROS node that connects the hardware to the Tobas software.

## Remote Connection

---

Configure the remote connection from the ground control station to the FC.

### Network Interface

Specify the network interface that the FC uses to communicate with external devices.
In this tutorial, pocket Wi-Fi is used for communication between the FC and GCS, so select `Wireless`.
If using wired LAN, select `Wired`.
If using the Raspberry Pi's built-in access point without an external communication module, select `Access Point`.
For other configurations, such as when using the VPN interface `tun0`, select `Other` and enter the interface name directly.

### Host

Configure the address that the GCS uses to identify the FC on the LAN.
You can use the FC's IP address directly if it is static, but this tutorial uses a convenient hostname instead.
Select `Hostname` and enter the FC hostname configured in [Boot Device Configuration](./bootmedia_config.md).

![remote_connection](../../assets/airframe_config/remote_connection.png)

## Saving the Project

Click `Save` and save the project as `tobas_f450.TBS` under `~/.local/share/tobas/colcon_ws/src/` in the file dialog.

![save](../../assets/airframe_config/save.png)

## Next Steps

---

The configuration is now complete.
Close Setup Assistant.
Next, assemble the aircraft and set up the hardware.
