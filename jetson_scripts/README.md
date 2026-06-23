# Jetson Scripts
The scripts in this directory are intended to be run on the Jetson itself, and are used for various setting up networking and serial connections. They typically should not be run inside the docker container but on the host itself, and will likely require sudo permissions.

## Networking scripts
These scripts are used to set up the network interfaces between the DVL and the Jetson when connected via a USB-to-Ethernet adapter. The DVL communicates to the Jetson via the [Waterlinked serial protocol](https://docs.waterlinked.com/dvl/dvl-protocol/#json-protocol-tcp), which provides a [Web GUI](https://docs.waterlinked.com/dvl/gui/dashboard/). The intended driver for this is the  [`dvl_a50`](https://github.com/mcgill-robotics/dvl_a50) ROS package, see the [Sensors README.md](../ros2_ws/src/sensors/README.md#waterlinked-dvl-a50) for more details.

### [`create_nmcli_connection.sh`](./networking/create_nmcli_connection.sh)
This script configures network connections for the DVL and router interfaces using `nmcli`, the CLI tool for NetworkManager. It creates a connection to communicate with the DVL device and ensures the it is set to low priority to avoid conflicts with the router connection.

### [`createBridge.sh`](./networking/createBridge.sh)
Create a level 2 bridge interface between the DVL and router interfaces. This allows the Jetson to communicate with both the DVL and the router simultaneously, enabling data exchange between the two networks. This notably allows any laptop connected to the jetson via SSH to access the DVL's Web GUI.

**Note that this script is know to cause some issues with the router connection and may kill SSH, so caution must be taken when running it.**

## Serial scripts
These scripts are used to set up a new serial connection with a device connected to the Jetson, notably used for the down camera, IMU and DVL with the [serial protocol](https://docs.waterlinked.com/dvl/dvl-protocol/#serial-protocol).

### [`detect_devices.sh`](./serial/detect_device.sh)
This scripts scans for typical pluggable devices and after pluggin it out then in, it will print out the new device that was added. This is useful for identifying the device path of a newly connected device, which can then be used to generate udev rules for it.

### [`generate_udev_rules.sh`](./serial/generate_udev_rules.sh)

This script generates udev rules for a given device path, which can be used to create a persistent symlink to the device. This is useful for ensuring that the device can be consistently accessed at the same path, even if it is plugged into a different USB port.

### Typical workflow for adding a new device
1. Run `detect_devices.sh` to identify the device path of the newly connected device.
2. Run `generate_udev_rules.sh` with the identified device path to generate the corresponding udev rules.
3. If the device communicates via a serial port, you can view the output via the `screen` command
```bash
screen <device_path> <baud_rate>
```
A good baud rate to start with is `115200`, but this may vary depending on the device. You can exit the `screen` session by pressing `Ctrl + A` followed by `K`, and then confirming with `Y`.