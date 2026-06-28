# Tobas User Guide

Tobas is a flight controller for drones and robotic aircraft based on model-based design and Linux.
Unlike conventional flight controllers, it designs control systems with detailed consideration of each airframe's structure,
allowing even aircraft that are difficult to fly with conventional flight controllers to be flown accurately.

## Features

---

- Defines airframe structures in a proprietary format and correctly reflects them in the controller
- Improved responsiveness and tracking performance through motor speed control
- Guarantees compensation for self-weight and reaction forces caused by movable joint angle changes
- Compensates for disturbances such as gusts and ground effect
- Fully GUI-based setup
- Supports ROS 2 and provides the same interface for simulation and real hardware

## What You Can Do

---

### Improved control performance

Because Tobas performs control using the physical characteristics of the user's airframe correctly,　
it can deliver better control performance than conventional flight controllers.
For example, it takes the following information into account:

- Dynamic parameters of each rigid-body link that makes up the airframe: mass, center of gravity, inertia tensor
- Battery specifications: cell count, discharge capacity, discharge rate
- Motor specifications: KV value, internal resistance, number of poles
- Propeller specifications: diameter, pitch, thrust coefficient, anti-torque coefficient

### Greater flexibility in airframe design

In Tobas, airframe structures are defined in [UADF (Universal Aircraft Description Format)](./additional_information/what_is_uadf.md),
making it possible to fly various aircraft that were difficult to handle with conventional flight controllers.
For example, Tobas supports unconventional aircraft such as:

- Aircraft whose center of gravity is significantly offset from the center due to payload
- Aircraft with asymmetrical propeller placement to secure a camera's field of view
- Aircraft equipped with a robotic arm
- Aircraft equipped with tilt rotors

### Reduced effort for gain tuning

By modeling the airframe correctly, the dynamics of the translational and rotational systems can be extracted in an airframe-independent form and analyzed in advance.
As a result, Tobas comes with reasonable default gains, and users can basically fly their aircraft without gain tuning.
If necessary, all parameters can also be adjusted online during flight.

### Realistic simulation

Because it takes into account the airframe's mass properties and the aerodynamic characteristics of the propulsion system, realistic physical simulation is possible.
This can significantly reduce the cost of real-world flight testing.

The following factors, which greatly affect flight, can be simulated easily:

- Wind (steady wind, turbulence, gusts)
- Battery voltage drop
- ESC maximum current
- Sensor delay and noise
- Suspended payload

## Flight Management Unit (FMU)

### Tobas FC201

<img src="../assets/introduction/fc201_1.jpg" alt="fc201_1" width="49%">
<img src="../assets/introduction/fc201_2.jpg" alt="fc201_2" width="49%">

#### Sensors & Processors

- 6-axis IMU: <a href=https://www.st.com/ja/mems-and-sensors/ism330dlc.html target="_blank">ISM330DLC | STMicroelectronics</a>
- Magnetometer: <a href=https://www.st.com/ja/mems-and-sensors/iis2mdc.html target="_blank">IIS2MDC | STMicroelectronics</a>
- Barometer: <a href=https://www.st.com/ja/mems-and-sensors/ilps22qs.html target="_blank">ILPS22QS | STMicroelectronics</a>
- GNSS Receiver: <a href=https://www.u-blox.com/en/product/zed-f9p-module target="_blank">ZED-F9P | u-blox</a>
- Voltage/Current Sensor: <a href=https://www.ti.com/product/ja-jp/INA228 target="_blank">INA228 | Texas Instruments</a>
- Main Computer: <a href=https://www.raspberrypi.com/products/raspberry-pi-5 target="_blank">Raspberry Pi 5</a>
- I/O Controller: <a href=https://www.st.com/ja/microcontrollers-microprocessors/stm32h7a3-7b3.html target="_blank">STM32H7A3 | STMicroelectronics</a>

#### Interface

- GNSS Antenna: SMA
- Power Module: Molex 2.0mm 6pin
- UART, I2C Interface: JST-GH 4pin

## Use Cases

---

### Quadcopter

A typical quadcopter.
It uses the DJI F450 frame kit.

<iframe width="560" height="315" src="https://www.youtube.com/embed/sHoA8yKJPs4?si=CCOEPsu6z9hd7zOb" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
<br>

### Hexacopter with non-coplanar rotor arrangement

A hexacopter with all propellers tilted 30 degrees from the horizontal plane.
While a multicopter with a coplanar rotor arrangement must change its attitude to change its position,
a multicopter with a non-coplanar rotor arrangement can control position and attitude independently.
This makes it possible to translate while maintaining an attitude parallel to the ground, or change attitude while hovering.
In addition, because it can generate horizontal thrust directly, it offers high positioning accuracy and excellent wind resistance.

<iframe width="560" height="315" src="https://www.youtube.com/embed/1RIXLGmx1RA?si=ADkOlZsAMb1tHyNr" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

### Active tilt hexacopter

A hexacopter whose all arms can rotate 120 degrees in both directions.
Its key feature is that it can change its attitude significantly while hovering.
Because attached inspection equipment can be held at any attitude, it is expected to be useful, for example, in non-destructive inspection of inclined walls.

<iframe width="560" height="315" src="https://www.youtube.com/embed/UYwoFjf6ubc?si=RsDKgr98DVvdhaWB" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
<br>

### Robot arm drone

An aircraft equipped with a 4-axis robotic arm mounted on a tilt hexacopter.
By dynamically compensating for reaction forces and shifts in the center of gravity generated by the arm,
it can keep its position and attitude within a certain range even when the arm is swung widely.

<iframe width="560" height="315" src="https://www.youtube.com/embed/3peWIltNV3o?si=OLdfuQGEHEI1L_N-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## System Requirements

---

### PC <!-- cf. https://www.solidworks.com/ja/support/system-requirements -->

The following requirements must be met.

| Requirement        | Required                       | Recommended        | Notes                          |
| :----------------- | :----------------------------- | ------------------ | ------------------------------ |
| OS                 | Ubuntu 24.04 LTS (ROS 2 Jazzy) |                    | Native environment recommended |
| RAM                | 8GB                            | 16GB               |                                |
| CPU                | AMD64 (x86-64)                 |                    |                                |
| GPU                |                                | NVIDIA GeForce RTX |                                |
| Display Resolution | 2K/FHD                         | 4K/UHD             |                                |

<br>

### ESC

It must support the Bidirectional DShot protocol.
For example, the following firmware supports it:

- <a href=https://github.com/bitdump/BLHeli/tree/master/BLHeli_32%20ARM target="_blank">BLHeli_32</a> (support ended in June 2024)
- <a href=https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware target="_blank">AM32</a>
- <a href=https://github.com/bird-sanctuary/bluejay target="_blank">bluejay</a>

### GNSS antenna

It must be compatible with the frequency band and connector of the GNSS receiver mounted on the FMU.

### RC receiver

It must support S.BUS with 8 channels or more.
