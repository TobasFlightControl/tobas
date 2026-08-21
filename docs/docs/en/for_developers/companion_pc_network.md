# Communication via a companion PC

This page explains how an external PC and the FC can continue to communicate over SSH and ROS 2
through a Linux companion PC such as an NVIDIA Jetson Nano.

This approach adds the onboard computing power needed for advanced processing such as image processing and SLAM without disrupting existing PC-to-FC communication.

## Network configuration

---

This configuration uses the companion PC as both a Wi-Fi client and a router.
IP forwarding and Proxy ARP forward packets from the external PC to the FC-side subnet, while Avahi relays mDNS traffic.

In the following, the FC-side subnet is `172.22.1.0/24`.
The companion PC uses `172.22.1.1` and assigns the FC an address from `172.22.1.100` through `172.22.1.200` via DHCP.

![Network configuration using the companion PC as a router](../../assets/companion_pc_network/network_topology.png)

| Function                                                    | Service         |
| ----------------------------------------------------------- | --------------- |
| Assigning an IP address to the FC                           | Dnsmasq         |
| Forwarding packets from the external PC to the FC           | IP Forwarding   |
| Making the FC appear directly connected to the external LAN | Proxy ARP       |
| Relaying the `.local` hostname                              | Avahi Reflector |

!!! warning

    If `172.22.0.0/16` is already in use on the LAN or VPN to which the external PC connects,
    it will conflict with the route added in this procedure.
    In that case, replace the settings below with a private address range that does not overlap any other network.

## Prerequisites

---

These instructions assume the following:

- The external PC and companion PC are on the same LAN and can communicate directly
- The companion PC uses NetworkManager to manage its network
- The companion PC and FC are connected via Ethernet
- ROS 2 Jazzy and Cyclone DDS are installed on the external PC

## Identify the interfaces to use

---

Check the Ethernet interface name and NetworkManager connection profile name on the companion PC.

If you are using a Jetson, first disable the virtual Ethernet interface created by USB Device Mode.
Do not run this command if the companion PC is not a Jetson.

```bash
$ sudo systemctl disable --now nv-l4t-usb-device-mode-runtime.service
```

Next, check the interfaces and addresses.

```bash
$ nmcli -f DEVICE,TYPE,STATE,CONNECTION device
```

From the output, find the line where `TYPE` is `ethernet`.

```txt
DEVICE             TYPE      STATE                   CONNECTION
...
eth0               ethernet  connected               Wired connection 1
...
```

On this line, the value of `DEVICE` is the interface name, and the value of `CONNECTION` is the connection profile name.

The following values are used below.
Replace them with the values for your environment.

| Item                                                    | Value used on this page |
| ------------------------------------------------------- | ----------------------- |
| Companion PC Wi-Fi interface name                       | `wlan0`                 |
| Companion PC Ethernet interface name                    | `eth0`                  |
| Companion PC NetworkManager Ethernet connection profile | `Wired connection 1`    |
| FC mDNS hostname                                        | `host1.local`           |

## Configure the companion PC

---

### Install the required packages

Install Dnsmasq and Avahi.

```bash
$ sudo apt update
$ sudo apt install -y dnsmasq avahi-daemon
```

### Set a static IP address for Ethernet

Set `172.22.1.1/24` in the Ethernet connection profile.
Replace `Wired connection 1` with the actual connection profile name.
To prevent this connection from becoming the default route, do not configure a gateway and enable `ipv4.never-default`.

```bash
$ sudo nmcli connection modify "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 172.22.1.1/24 \
    ipv4.gateway "" \
    ipv4.never-default yes
```

### Configure the DHCP server

To assign an IP address to the FC, create `/etc/dnsmasq.d/tobas-fc.conf`.
Replace `eth0` with the actual Ethernet interface name.

```ini
interface=eth0
bind-dynamic
dhcp-range=172.22.1.100,172.22.1.200,255.255.255.0,12h
```

### Enable IP forwarding and Proxy ARP

After the route described below is configured, the external PC sends packets destined for the FC as though the FC were on the same LAN.
With Proxy ARP enabled, the companion PC responds to the external PC on behalf of the FC
and receives packets destined for the FC.
This makes the FC appear to be directly connected to the external LAN.

IP forwarding passes packets received by the companion PC
between the external LAN interface and the FC-side Ethernet interface.
To enable these features, create `/etc/sysctl.d/99-tobas-routing.conf`.
Replace `wlan0` with the actual interface name on the external LAN side.

```ini
net.ipv4.ip_forward=1
net.ipv4.conf.wlan0.proxy_arp=1
```

### Enable the mDNS reflector

To relay the `.local` hostname advertised by the FC to the external LAN, enable the reflector in the `[reflector]` section of `/etc/avahi/avahi-daemon.conf`.
If an `[reflector]` section already exists, edit the existing value instead of adding a new section.

```ini
[reflector]
enable-reflector=yes
```

## Configure a route to the FC-side subnet on the external PC

---

To automatically configure a route to the FC-side subnet on the Wi-Fi or Ethernet NIC used for the external PC's current default route,
create `/etc/NetworkManager/dispatcher.d/90-tobas-route`.

```sh
#!/bin/sh

# Run only when the interface is activated or the DHCP information is updated.
ACTION="$2"
[ "$ACTION" = "up" ] || [ "$ACTION" = "dhcp4-change" ] || exit 0

# Get the NIC used by the current default route.
IFACE="$(ip route get 1.1.1.1 | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')"

# Do nothing if no default route exists.
[ -n "$IFACE" ] || exit 0

# Get the NIC type.
TYPE="$(nmcli -g GENERAL.TYPE device show "$IFACE" 2>/dev/null)"

# Do nothing unless the NIC is Wi-Fi or Ethernet.
[ "$TYPE" = "wifi" ] || [ "$TYPE" = "ethernet" ] || exit 0

# Configure the route to the FC-side subnet on the NIC used by the current default route.
ip route replace 172.22.0.0/16 dev "$IFACE"
```

Make the script an executable file owned by root.

```bash
$ sudo chmod 755 /etc/NetworkManager/dispatcher.d/90-tobas-route
$ sudo chown root:root /etc/NetworkManager/dispatcher.d/90-tobas-route
```

## Verify communication from the external PC to the FC

---

Once the steps above are complete, restart the FC, companion PC, and external PC.
If the settings have been applied correctly, the external PC and FC should now be able to communicate through the companion PC.

### Verify the SSH connection

No special SSH tunnel or port forwarding is required.
You can specify the FC's mDNS hostname as you would for a normal SSH connection.

```bash
$ ssh pi@host1.local
```

### Verify ROS 2 communication

On routed networks, the default multicast-based ROS 2 discovery traffic does not pass between the external PC and the FC.
Therefore, set the FC's hostname in `ROS_STATIC_PEERS` on the external PC so that Cyclone DDS can discover the FC via unicast.

```bash
$ source /opt/ros/jazzy/setup.bash
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ export ROS_STATIC_PEERS=host1.local
$ ros2 daemon stop
$ ros2 topic list
```

If the FC's topics are displayed, discovery is working correctly.
Also verify that messages can actually be received.

```bash
$ ros2 topic echo <Topic Name>
```

## Advanced

---

### Assign a static IP address to the FC

The configuration above uses an mDNS hostname to identify the FC, but you can use a static IP address instead.

Before booting the FC, use [Tobas Bootmedia Config](../getting_started/bootmedia_config.md) to write a static IP address to the boot device.
On the `IP Address` tab, configure the following under `Wired`.

| Item            | Value                |
| --------------- | -------------------- |
| `Method`        | `Manual`             |
| `Prefix Length` | `24 - 255.255.255.0` |
| `Address`       | `172.22.1.2`         |
| `Gateway`       | `172.22.1.1`         |

After clicking `Write` to write the settings, click `Disconnect` to eject the boot device, then boot the FC.
Setting `Gateway` to the companion PC's address, `172.22.1.1`, allows the FC to return response packets to the external PC.

In this case, you do not need to configure the DHCP server on the companion PC.
The mDNS reflector is also unnecessary if you specify `172.22.1.2` directly for SSH and in `ROS_STATIC_PEERS`.

```bash
$ ssh pi@172.22.1.2
$ export ROS_STATIC_PEERS="172.22.1.2"
```

### Operate multiple vehicles on the same LAN

If each companion PC uses a unique FC-side subnet and each FC has a unique hostname, a single external PC can communicate with multiple vehicles.
For example, configure three vehicles as follows:

| Vehicle | FC-side subnet  | Companion PC | DHCP range                    | FC hostname   |
| ------- | --------------- | ------------ | ----------------------------- | ------------- |
| A       | `172.22.1.0/24` | `172.22.1.1` | `172.22.1.100`–`172.22.1.200` | `host1.local` |
| B       | `172.22.2.0/24` | `172.22.2.1` | `172.22.2.100`–`172.22.2.200` | `host2.local` |
| C       | `172.22.3.0/24` | `172.22.3.1` | `172.22.3.100`–`172.22.3.200` | `host3.local` |

For ROS 2 communication with multiple FCs, specify static peers separated by semicolons.

```bash
$ export ROS_STATIC_PEERS="host1.local;host2.local;host3.local"
```
