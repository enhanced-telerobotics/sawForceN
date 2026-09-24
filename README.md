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
