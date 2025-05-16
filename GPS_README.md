# Gazebo GPS Plugin

A lightweight, ROS-enabled Gazebo plugin that simulates a GPS receiver by converting ENU coordinates to latitude, longitude, and altitude, with configurable update rate and noise model.


## Features

- **ENU→GPS Conversion**  
  Converts model’s East-North-Up (Gazebo) pose into WGS84 latitude, longitude, altitude.
- **Configurable Update Rate**  
  Publish GPS fixes at a user-defined frequency (default 5 Hz).
- **Gaussian Noise Model**  
  Add realistic sensor noise via configurable standard deviation.
- **ROS Integration**  
  Publishes `sensor_msgs/NavSatFix` over ROS; respects `rosNamespace`.
- **Simple SDF Parameters**  
  All key parameters exposed in SDF for easy tuning (update rate, reference origin, noise, frame and topic names).
- **Zero External Dependencies**  
  Relies only on Gazebo, ROS, and standard C++ STL.


## Installation

1. **Clone & Build**  
   ```bash
   cd your_catkin_ws/src
   git clone https://github.com/dringakn/uav_gazebo.git
   cd ..
   catkin build uav_gazebo
   ```

2. **Source & Run**
   ```bash
   source devel/setup.bash
   roslaunch your_uav_simulation bringup.launch
   ```

## Usage

### SDF Snippet

```xml
<plugin name="gps_plugin" filename="libuav_gazebo_gps_plugin.so">
  <!-- ROS namespace for topics and services -->
  <rosNamespace>/uav</rosNamespace>

  <!-- Publishing frequency (Hz) -->
  <updateRate>5.0</updateRate>

  <!-- Reference WGS84 origin (ENU frame) -->
  <referenceLatitude>47.397742</referenceLatitude>
  <referenceLongitude>8.545594</referenceLongitude>
  <referenceAltitude>300.0</referenceAltitude>

  <!-- Gaussian noise standard deviation (meters in ENU) -->
  <noiseStdDev>1.0</noiseStdDev>

  <!-- Header frame and topic -->
  <frameId>gps_link</frameId>
  <topicName>/uav/gps</topicName>
</plugin>
```

### Parameter Descriptions

| Parameter            | Type   | Default      | Description                                                     |
| -------------------- | ------ | ------------ | --------------------------------------------------------------- |
| `rosNamespace`       | string | `""`         | ROS node namespace for publisher                                |
| `updateRate`         | double | `5.0`        | GPS fix publish rate in Hz                                      |
| `referenceLatitude`  | double | `0.0`        | WGS84 latitude at Gazebo ENU origin                             |
| `referenceLongitude` | double | `0.0`        | WGS84 longitude at ENU origin                                   |
| `referenceAltitude`  | double | `0.0`        | Altitude (m) at ENU origin                                      |
| `noiseStdDev`        | double | `0.0`        | Standard deviation of Gaussian noise (m in ENU) |
| `frameId`            | string | `"gps_link"` | `header.frame_id` in published `NavSatFix`                      |
| `topicName`          | string | `"/gps"`     | ROS topic name for `NavSatFix`                                  |


## Example

```bash
# Launch Gazebo with your UAV model
roslaunch uav_gazebo_plugins demo.launch
# In another terminal, echo GPS fixes
rostopic echo /uav/gps/fix
```


## Limitations & Caveats

* **Flat Earth Assumption**
  Uses simple spherical Earth conversion — error grows over multi‐kilometer distances.
* **No SBAS/RTK**
  Doesn’t support augmentation, dynamic reference shifts, or carrier‐phase corrections.
* **Constant Variance**
  Noise is stationary Gaussian; no time‐varying drift or outliers.
* **Single Reference Origin**
  Only one fixed ENU→WGS84 origin per plugin instance.


## Tips & Best Practices

* **High-Frequency Simulations**: Keep `updateRate` ≤ simulation physics rate to avoid backlog.
* **Noise Tuning**: For MAV demonstration, `noiseStdDev` ≈ 0.2–1.0 m gives realistic wandering.
* **Multiple GPS Units**: Duplicate plugin blocks with different `frameId`/`topicName` for sensor fusion tests.


## License

MIT License — see [LICENSE](LICENSE) for details.


## Author

Developed by Dr. -Ing. Ahmad Kamal Nasir – opinions and suggestions welcome!

