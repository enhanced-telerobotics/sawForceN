# sawForceN / force_n

ROS 2 node `force_n` for the ASY-00032/3-001 sensor documented in MAN-00017-001.
Standalone Python/PyQt implementation inspired by sawATIForceSensor's Sensor Stats,
Interval Stats and Rebias controls; does not depend on cisst. Calibrated forces only.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/erie_lab/ros2_ws
colcon build --packages-select force_n --symlink-install
source install/setup.bash
ros2 run force_n force_n --port /dev/ttyACM0
```

Use the system Python and ROS environment (rclpy, python3-serial, python3-pyqt5),
not the separate `force` virtual environment. Add `--no-gui` for headless operation.
`--ros-args -r __ns:=/force_sensor_A` changes the namespace.
Parameters: `serial_port`, `baud_rate` (115200), `frame_id` (`force_n_sensor`),
`tare_timeout` (60 seconds). CLI equivalents: `--port`, `--baud`, `--frame-id`.

- `measured_cf`: geometry_msgs/msg/WrenchStamped, forces in N (device mN / 1000),
  torque x/y/z zero in N·m. Axes follow the sensor coordinate system; no TF rotation.
  Header stamps use ROS publication time; the sensor supplies no timestamp.
- `status`: std_msgs/msg/String, connection/tare/error state.
- `rebias`: std_srvs/srv/Trigger; accepts a hardware tare request asynchronously.
  Success means accepted, not completed. Monitor `status` for Streaming or Error.

```bash
ros2 topic echo /measured_cf
ros2 service call /rebias std_srvs/srv/Trigger '{}'
```

GUI Rebias and ROS rebias both stop the stream and send `<SDM3>`. `e0x1` means
started; `e0x4` means completed. Only then does `<SDM2>` resume streaming. No forces
are published while taring, on disconnect, or after a protocol error. Failure stops
acquisition; restart the node after resolving it. The GUI clears stale forces.
No software bias or EEPROM save (`<CS>`) is used. Startup selects calibrated output
using `<GI1>`, `<SEL...>`, `<TDO2>`, `<TOS0x10>`; this changes the volatile output mode.
Closing the GUI / Ctrl+C stops streaming and closes serial. Do not run reader.py
or another sensor node on the same port concurrently.

Manual §9: warm up for at least five minutes, tare unloaded and stationary, and
re-tare after orientation changes or power cycling. Tare is explicit, never automatic.
The device has no documented torque output or saturation flag in this protocol;
zero torque is a placeholder, not a measurement. Interval statistics reflect host
receive timing. A bounded queue retains up to 2000 pending samples; prolonged ROS
stalls can drop oldest samples. No automatic reconnect is performed.

## Persistent sensor names on a new PC

[udev/99-forcen.rules](udev/99-forcen.rules) defines persistent device aliases for
tools owned by CWRU ERIE Lab (Case Western Reserve University). The rules match
each tool's USB serial number, so the alias follows that tool across reboots,
connection order changes, and USB ports. The underlying `/dev/ttyACM*` names may
still change.

| Device alias | USB serial number |
| --- | --- |
| `/dev/CLASSIC_LND` | `2084358A4242` |
| `/dev/S_LND` | `206B35894242` |
| `/dev/CLASSIC_MBF` | `209F35894242` |
| `/dev/S_MBF` | `205D358E4242` |

On a new Ubuntu/ROS 2 PC, run these commands from the root of this repository:

```bash
sudo install -m 0644 udev/99-forcen.rules /etc/udev/rules.d/99-forcen.rules
sudo udevadm control --reload-rules
```

Stop any running sensor nodes, then unplug and reconnect the sensors. Check the
aliases for the tools currently connected (disconnected tools have no alias):

```bash
ls -l /dev/CLASSIC_* /dev/S_*
```

If your account does not already have serial-port access, add it to `dialout`,
then log out and log back in for the group membership to take effect:

```bash
sudo usermod -aG dialout "$USER"
```

Use an alias when starting a sensor node, for example:

```bash
ros2 run force_n force_n --port /dev/CLASSIC_LND
```

The launch files below select these aliases using tool arguments. Install the
udev rules before using them.

To register another tool, inspect its current device with
`udevadm info --attribute-walk --name=/dev/ttyACM0` (adjust the device number).
Add a rule to `udev/99-forcen.rules` using its USB parent device's `idVendor`,
`idProduct`, and unique `serial`, and choose an unused alias. The serial numbers
above apply only to the listed lab tools. After editing the file, repeat the
installation, reload, and reconnect steps on each PC using the tools.

## Single-arm launch

```bash
ros2 launch force_n single_arm.launch.py
```

Defaults to `arm:=PSM1 tool:=CLASSIC_LND`, opening `/dev/CLASSIC_LND` in
namespace `/PSM1/shaft` with the GUI.
Publishes `/PSM1/shaft/measured_cf` and `/PSM1/shaft/status`; hardware tare is available
at `/PSM1/shaft/rebias`.

Select another arm and tool without editing the launch file:

```bash
ros2 launch force_n single_arm.launch.py arm:=PSM3 tool:=S_LND
```

Allowed arms are `PSM1`, `PSM2`, and `PSM3`. Allowed tools are `CLASSIC_LND`,
`S_LND`, `CLASSIC_MBF`, and `S_MBF` (case-sensitive).

## Dual-arm launch

```bash
ros2 launch force_n dual_arm.launch.py
```

Starts two sensor nodes and GUIs: `CLASSIC_LND` on `PSM1` and `CLASSIC_MBF` on
`PSM2`. Each publishes and exposes rebias under its own `/<arm>/shaft` namespace.
Each GUI identifies its arm and tool in the window title and a persistent header
above both tabs, with the device path and ROS namespace shown underneath.
Use `arm1`/`tool1` and `arm2`/`tool2` to override either pairing:

```bash
ros2 launch force_n dual_arm.launch.py tool1:=S_LND tool2:=S_MBF
ros2 launch force_n dual_arm.launch.py arm1:=PSM2 tool1:=S_LND arm2:=PSM3 tool2:=S_MBF
```

The allowed arms and tools are the same as for single-arm launch. Duplicate arms
or tools are rejected before starting either node, preventing namespace collisions
and two readers opening the same sensor.

After updating this repository, rebuild `force_n` and source the workspace's
`install/setup.bash` as shown above so both launch files are available.
