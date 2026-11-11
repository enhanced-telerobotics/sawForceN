# sawForceN / force_n

ROS 2 driver and Qt GUI for the Forcen ASY-00032/3-001 calibrated force sensor
(MAN-00017-001). Automatically matches sensors to dVRK PSM tool selections.

## Build

Use system Python with ROS 2 Jazzy, `rclpy`, `python3-serial`, and `python3-pyqt5`.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select force_n --symlink-install
source install/setup.bash
```

Rebuild and source the workspace after updates.

## Windows (PowerShell)

This is an `ament_python` package; no C++ compiler is required. On this PC,
ROS 2 Jazzy, Python 3.12 and PyQt5 are installed under `C:\pixi_ws`.
Use the existing ROS environment script and a regular build without
`--symlink-install`:

```powershell
. 'C:\Users\ERIE_Lab\bin\rosenv.ps1'
cd C:\ros2_ws
python -m pip install pyserial
colcon build --packages-select force_n
. 'C:\Users\ERIE_Lab\bin\rosenv.ps1'
ros2 launch force_n psm_sensors.launch.py
```

Add `gui:=false` for headless operation. In subsequent terminals, source
`rosenv.ps1` and run the launch command. The package is installed under
`C:\ros2_ws\install\force_n`; no repository-local activation script or virtual
environment is needed.

On Windows, the numeric device aliases in the table below resolve by USB
serial number to their current COM ports; no udev installation is needed.
Direct startup defaults to `/dev/400906`
on Windows and `/dev/ttyACM0` on Linux. Missing or ambiguous sensors produce
an error. To inspect devices or manually select a port:

```powershell
python -m serial.tools.list_ports -v
ros2 run force_n force_n --port /dev/400906
ros2 run force_n force_n --port COM3 --no-gui
python -m pytest -q test
```

COM numbers can change; aliases identify the lab sensors reliably. Stop the
manager before starting manual readers.

## Sensor setup (Linux)

Install [udev/99-forcen.rules](udev/99-forcen.rules) from this repository's root
on each PC:

```bash
sudo install -m 0644 udev/99-forcen.rules /etc/udev/rules.d/99-forcen.rules
sudo udevadm control --reload-rules
```

Stop sensor readers, unplug and reconnect the sensors, then check
`ls -l /dev/400* /dev/420*`. If serial access is denied, run
`sudo usermod -aG dialout "$USER"`, then log out and back in.

The aliases identify the lab's physical sensors by USB serial number:

| Physical tool | Tool-index device alias | USB serial number |
| --- | --- | --- |
| LND 400006 | `/dev/400906` | `2084358A4242` |
| LND 420006 (non-geared) | `/dev/420906` | `206B35894242` |
| MBF 400172 | `/dev/400972` | `209F35894242` |
| MBF 400172E | `/dev/400872` | `205D358E4242` |

Device aliases use the custom ForceN model numbers defined in dVRK's tool
index, as shown above. Both MBFs are Classic instruments; `400872` identifies
the E-marked tool. Reinstall the rules, reload, and reconnect after alias changes.

## Automatic PSM matching

Start dVRK with the appropriate system configuration, then run:

```bash
ros2 launch force_n psm_sensors.launch.py
```

Add `gui:=false` for headless operation. The manager watches
`/PSM1/tool_type`, `/PSM2/tool_type`, and `/PSM3/tool_type`, including retained
selections published before startup. A matching tool starts its sensor under
`/<PSM>/shaft`. Moving the tool to another PSM stops the old reader before
starting the new one. Removal, `ERROR`, an unregistered tool, or disappearance
of the tool publisher from the ROS graph stops that arm's reader.

Supported tool IDs are defined once in `TOOL_MODELS` in
[saw_force_n/tool_manager.py](saw_force_n/tool_manager.py). The default mapping
uses each model number as its device alias. Register each ID in dVRK's custom
index without instance version fields. For example, `MBF_FORCEN:400872[..]`
selects `/dev/400872`. No parameter file is needed.

Each sensor has a distinct custom model. Explicit revision suffixes require an
exact revision mapping; obsolete instance IDs such as `400972[2..2]` are rejected.
Restart dVRK and the manager after migrating the IDs. Run one manager and stop
manual readers first. If two PSMs report the same sensor ID, that reader stops
until the conflict is resolved. Both MBFs can run simultaneously.
Closing the manager stops its readers and GUIs. A closed or exited reader waits
for an assignment change or manager restart; serial errors require a restart
after resolving the problem. There is no automatic reconnect.

## ROS interface and rebias

For a sensor matched to PSM1:

| Topic / service | Type | Meaning |
| --- | --- | --- |
| `/PSM1/shaft/measured_cf` | `geometry_msgs/msg/WrenchStamped` | Calibrated force in N |
| `/PSM1/shaft/status` | `std_msgs/msg/String` | Connection, tare, or error state |
| `/PSM1/shaft/rebias` | `std_srvs/srv/Trigger` | Request hardware tare |

```bash
ros2 topic echo /PSM1/shaft/measured_cf
ros2 service call /PSM1/shaft/rebias std_srvs/srv/Trigger '{}'
```

Force axes follow the sensor frame; no TF rotation is applied. Timestamps are
ROS publication time. Torque fields are zero placeholders, not measurements.

Warm up for at least five minutes and tare unloaded and stationary. Re-tare
after orientation changes or power cycling. GUI Rebias and the ROS service
request the same hardware tare; service success means accepted, not completed.
Monitor `status` for Streaming or Error. Tare is explicit, never automatic, and
force publication pauses during tare or errors. No software bias or EEPROM
save is used. Closing the GUI or pressing Ctrl+C stops streaming and closes serial.
