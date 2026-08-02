# Configuring the Boot Device

First, edit the boot device directly to configure the initial communication settings.
Prepare a microSD card containing the Tobas image.

## Preparation

---

### Launching the Configuration GUI

Run the following command in a terminal to launch Tobas Bootmedia Config.
Root privileges are required because this tool accesses external volumes.

```bash
$ sudo env XDG_CONFIG_HOME="$HOME/.config" bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run tobas_bootmedia_config TobasBootmediaConfig'
```

![start](../../assets/bootmedia_config/start.png)

### Connecting to the Boot Device

Insert the SD card into the PC using a suitable USB card reader.
If the Tobas image has been written correctly, the SD card will appear in the selection list at the top of the GUI.

Make sure that the correct SD card is selected, then click `Connect`.
The SD card will be mounted on the PC, and its current settings will be loaded.

![connect](../../assets/bootmedia_config/connect.png)

## Settings

---

### Hostname

Edit the Linux hostname.

By default, Tobas uses hostnames to identify devices on the network, so each hostname must be unique within the intended subnet.
Devices can also be identified using static IP addresses, but when using multiple units simultaneously, we recommend assigning each one a unique hostname to avoid conflicts.

Edit the hostname and click `Write` to write the new hostname to the SD card.

![hostname](../../assets/bootmedia_config/hostname.png)

### Login Password

Edit the login password.

The default password, `tobas`, will work, but changing it is recommended for security reasons.

Enter the login password in `New Password`, then enter the same password in `Confirm Password` for confirmation.
If the passwords match and are valid, the `Write` button will be enabled.
Click `Write` to write the password to the SD card.

![login_password](../../assets/bootmedia_config/login_password.png)

### SSH Keys

Configure SSH key authentication.

Tobas uses SSH key authentication for some operations performed on the flight controller (FC) from the ground station.
You must therefore register the public key of the PC used as the ground station with the FC.

First, create an SSH key.
Run the following command in a terminal to launch `Passwords and Keys`.

```bash
$ seahorse
```

From the `+` button in the upper-left corner, select `Secure Shell key`.
In the dialog that appears, enter an identifier such as `<ユーザ名>@<ホスト名>` in `Description`, then click `Generate`.
In the next dialog, click `OK` to generate an SSH public and private key pair. You can leave the password blank.
Then click `OpenSSH keys` and confirm that the generated key is displayed.
Double-click the key, make a note of `Public key` in the dialog that appears, then close `Passwords and Keys`.

<img src="../../../assets/bootmedia_config/ssh_key_1.png" alt="ssh_key_1" width="49%"/>
<img src="../../../assets/bootmedia_config/ssh_key_2.png" alt="ssh_key_2" width="49%"/>

Return to `Tobas Bootmedia Config`, click `Add`, then copy and paste the public key you noted earlier into the dialog that appears.
Click `OK` to add the public key to the list and write it to the SD card.

![ssh_keys](../../assets/bootmedia_config/ssh_keys.png)

### Wi-Fi Client

Configure the FC to operate as a Wi-Fi client.

Tobas uses ROS 2 (DDS) for remote communication.
The FC and all devices that need to communicate with it, such as the ground station, must belong to the same subnet.
You can skip this section if you only use Ethernet or operate the FC as an access point.

Click `Add` and enter the SSID and PSK of the access point you want to connect to in the dialog that appears.
`Priority` specifies the connection priority. If multiple networks are available, networks with higher values are given priority.
Click `OK` to add the access point to the table and write it to the SD card.

![wifi_client](../../assets/bootmedia_config/wifi_client.png)

### Wi-Fi Hotspot

Configure the FC to operate as a Wi-Fi access point.

As described above, the FC and ground station must belong to the same subnet to communicate.
Configuring the FC itself as an access point is convenient for outdoor test flights because no external router is required.
You can skip this section if the FC will only operate as a Wi-Fi client.

Enter the desired SSID and PSK in `New SSID` and `New PSK`, respectively.
If both values are valid, the `Write` button will be enabled.
Click `Write` to write the SSID and PSK to the SD card.

![wifi_hotspot](../../assets/bootmedia_config/wifi_hotspot.png)

## Finishing Up

---

Once you have completed the settings above, click `Disconnect`.
The boot device will be unmounted and can then be safely removed.
Close the GUI and remove the SD card from the PC.

![disconnect](../../assets/bootmedia_config/disconnect.png)

## Next Steps

---

The configuration is now complete.
Next, use Tobas Setup Assistant to create your first project.
