# Hardware Setup

## Assembling the Vehicle

---

Assemble the vehicle as configured in Setup Assistant.

<!-- TODO: Navio2のような詳細な手順 -->
<!-- cf. https://docs.emlid.com/navio2/hardware-setup/ -->
<!-- cf. https://docs.emlid.com/navio2/ardupilot/typical-setup-schemes/ -->

<img src="../../../assets/hardware_setup/f450_1.png" alt="f450_1" width="49%"/>
<img src="../../../assets/hardware_setup/f450_2.png" alt="f450_2" width="49%"/>

<!-- prettier-ignore-start -->
!!! note
    When mounting the flight controller on the vehicle, be sure to provide at least basic vibration isolation.
    If the mounting is too rigid, vibrations from the motors and propellers may make acceleration readings noisy and reduce attitude estimation accuracy.
    Conversely, if it is too soft, delayed gyro measurements may cause oscillations in angular velocity control.
    For readily available options, we recommend
    <a href=https://holybro.com/products/foam-pads-20pcs target="_blank">Holybro Foam Pads</a> or
    <a href=https://rc.kyosho.com/ja/z8006b.html target="_blank">Kyosho Z8006B</a>.
<!-- prettier-ignore-end -->

## Transmitter Setup

---

The S.BUS signal is expected to provide at least eight channels.
In Tobas, each RC input channel has the following role:

| Channel | Function        | Interface         |
| :------ | :-------------- | :---------------- |
| CH1     | Roll            | Stick             |
| CH2     | Pitch           | Stick             |
| CH3     | Throttle        | Stick             |
| CH4     | Yaw             | Stick             |
| CH5     | Flight mode     | 3-position switch |
| CH6     | Sub-flight mode | 2-position switch |
| CH7     | Enable/Disable  | 2-position switch |
| CH8     | Kill            | 2-position switch |
| CH9-16  | GPSw            | 2-position switch |

<br>

The sub-flight mode switch provides finer mode selection within specific flight modes.
It is used when the four primary stick axes do not provide enough command degrees of freedom, such as when changing modes on a vehicle capable of controlling five or more axes simultaneously.
GPSw (General Purpose Switch) is a switch that users can assign freely.
You can configure the number of switches in Setup Assistant according to the transmitter, receiver, and intended use.

The <a href=https://www.rc.futaba.co.jp/products/detail/I00000006 target="_blank">Futaba T10J</a>
has fixed assignments for channels 1 through 4 as shown in the table above,
while switches can be assigned freely to channels 5 and above.
Press and hold the `+` button on the transmitter to open the menu, then select `AUXチャンネル`.
For this example, the channels were configured as follows.

| Channel | Switch |
| :------ | :----- |
| CH5     | SwE    |
| CH6     | SwG    |
| CH7     | SwA    |
| CH8     | SwC    |
| CH9     | NULL   |
| CH10    | NULL   |

<br>

When using a Futaba transmitter, you must also reverse the throttle stick.
Press and hold the `+` button on the transmitter to open the menu, then select `リバース`.
Set only the throttle stick (`THR`) to reverse (`REV`).

## Loading and Writing a Tobas Project

---

Connect the ground station PC to the same network as the FC.
If you configured multiple networks in [Boot Device Configuration](./bootmedia_config.md),
note that the highest-priority available network will be selected.

Run the following command in a terminal to launch Tobas GCS.

```bash
$ ros2 launch tobas_gcs gcs.launch.py
```

Click `Load Project`, then double-click `tobas_f450.TBS` created in Setup Assistant to load it.
Clicking `Write Project` sends the project to the FC and then builds it. This takes several minutes.

![load_and_write](../../assets/hardware_setup/load_and_write.png)

## Sensor Calibration

Calibrate each sensor.
Click `Sensor Calib` in the toolbar at the top of the screen.

---

### Accelerometer Calibration

Calibrate the accelerometer.
Place the vehicle on a level surface and click `Start`.
Calibration completes in a few seconds, and the tab changes from red to green shortly afterward.

![accel_calibration](../../assets/hardware_setup/accel_calibration.png)

### Magnetometer Calibration

Calibrate the magnetometer.
Because the magnetometer is highly affected by its surroundings, perform the calibration with the FC mounted on the vehicle.
It is also best to perform it away from magnetic materials such as steel reinforcement.

1. Click `Start`. The magnetometer readings will begin to appear as a white point cloud.
1. For each of the FC's six faces, orient the face upward and slowly rotate the vehicle about the vertical axis.
   This step is complete when the progress bar reaches 100%.
1. When finished, click `Finish`. The estimated ellipsoid appears in blue, and the distortion-corrected point cloud appears in green.
   Calibration is successful if the blue ellipsoid overlaps the white point cloud and the green point cloud forms a sphere around the origin.

<img src="../../../assets/hardware_setup/mag_calibration_1.png" alt="mag_calibration_1" width="49%"/>
<img src="../../../assets/hardware_setup/mag_calibration_2.png" alt="mag_calibration_2" width="49%"/>

### Radio Calibration

Calibrate the radio input (S.BUS).

1. Click `Start`. The value of each S.BUS channel will begin to appear.
1. For each channel, move the stick or switch through its full range.
   If the stick and GUI bar move in opposite directions, adjust the transmitter settings accordingly.
1. When finished, click `Finish`.

![radio_calibration](../../assets/hardware_setup/radio_calibration.png)

## Actuator Test

Test each actuator.
Click `Actuator Test` in the toolbar at the top of the screen.

---

### Rotor Test

<span style="color: red;"><strong>Warning: This operation spins the motors. Exercise extreme caution if performing it with the propellers installed.</strong></span>

![rotor_test](../../assets/hardware_setup/rotor_test.png)

1. Click `Start` to enable all motors.
1. For each motor, move the slider to command its speed, then verify that it is connected correctly and rotates in the correct direction.
1. Adjust the control gain for each motor.
   Increase the gains one at a time while changing the target speed and checking that no oscillation occurs.
   In this example, all gains were set to 17.
1. Click `Save` to save the control gains to the project on the PC.
1. Click `Stop` to end the test.
1. Click `Write Project` to flash the saved control gains to the FC.

<!-- prettier-ignore-start -->
!!! note
    If the motors do not spin, check whether the `ERR` indicator on top of the FC is lit.
    If it is lit, turn the power off and then power the FC on again.
<!-- prettier-ignore-end -->

### Joint Test

If the vehicle has PWM-driven joints, such as tilt-rotor mechanisms or fixed-wing control surfaces, you can test the position commands for each joint.
This vehicle has no movable joints other than the propellers, so this test is skipped.

## Powering Off the FC

---

1. Press the red power button in the upper-right corner of the screen to shut down the FC.
1. Confirm that the Raspberry Pi power button has changed from green to red before disconnecting the power.

![power_off](../../assets/hardware_setup/power_off.png)

## Next Step

---

This completes the setup.
In the next step, you will finally fly the vehicle.
