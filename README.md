# uav_gazebo

Plugin for simple simulation of a quad-copter in Gazebo and ROS.

**Tested with ROS Noetic**
### Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/dringakn/uav_gazebo.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## An example for the impatients

Start Gazebo/RViz/MissionManager (with ROS connection):
If a PS4 Joystick is attached to the system the drone can
be controlled using the joystick controller

```
roslaunch uav_gazebo bringup.launch
```

    PS4 controller interface:
    - Left Stick: X, Y
    - Right Stick: Z, Yaw
    - R2/R2: takeoff/land
    - L1: THRUST_TORQUE Mode
    - R1: INACTIVE, STOP, SWITCHOFF, DROP
    - CROSS: POSITION_YAW Mode
    - CIRCLE: VELOCITY_YAWRATE Mode
    - SQUARE: THRUST_ATTITUDE Mode
    - TRIANGLE: THRUST_VELOCITY Mode
    - LEFT/RIGHT STICK BUTTON: RESET DRONE POSITION

If the drone is in VELOCITY_YAWRATE (CIRCLE) it will be controlled by the movement of control sticks. If the drone is in POSITION_YAW (CROSS) mode then the control sticks movenment can be seen as a target position. If the R1 is pressed then the drone will drope (similar to a kill switch), it can be lifted off by pressing VELOCITY_YAWRATE (CIRCLE) and with Right-Stick Up movement. The drone position can be reset (teleported) to
fixed position using LEFT or RIGHT Stick button press.
Press SHARE button to reset drone position as well as pursuit controller (start if already not running).

If no joystick is present then open a second termainal and execute a simple test controller

```
rosrun uav_gazebo example_control
```

Similarly if no joystick is present, the drone mode can be change by a service call. For example for the testing of a pure pursuit controller, make sure the drone is in mode 2 (velocity_yawrate control mode) by executing the following service:

```
rosservice call /drone/switch_mode "{mode: {mode: 2}}"
```

Afterwards execute following command in a terminal to start sending pursuit controller commands:

```
rosrun uav_gazebo pursuit_controller
```

## Packages

- `uav_gazebo`: catkin package containing the actual Gazebo plugin plus some example code.
- `uav_gazebo_msgs`: catkin package defining custom messages used by the plugin.

## Plugin description

The plugin is rather simple. The drone is assumed to be a floating rigid body, which can be moved around by apllying a generic torque and a force aligned to the z-axis of the body. The plugin can be attached to a model in the usual way, and will use the root link as the body that fully represents the drone.

### Dynamic parameters

The plugin will automatically collect the mass and inertia of the rigid body it is attached to, and use them as parameters for the dynamic model. There is however one restriction: the center of mass of the link must coincide with the origin of the link itself. This is because in the controllers, translational and rotational dynamics are considered to be decoupled. Note that in principle the reference frame of the inertia needs not to be aligned with the link frame, although this scenario has not be tested.

### Control modes

The plugin allows to control the drone in different control modes (they are discussed more in details in the [dedicated section in the theoretical background](#control-schemes)):

- Position and yaw
- Velocity and yaw rate
- Thrust and attitude
- Thrust and angular velocity
- Thrust and torque

In addition, the drone can be set in "idle mode". Note that the drone starts in this mode.

### ROS interface

To receive control inputs, the drone is connected with several ROS topics. In addition, it will provide some feedback on its state via some publishers. All ROS connections are "located" in a namespace that can be provided to the plugin via an XML tag, `rosNamespace`. It is recommended to set such namespace to something unique, such as the name of the drone, to allow control of multiple models at the same time.

#### Published topics

- `odometry` (type [`nav_msgs/Odometry`](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)): used to provide the current pose and twist of the drone. Note that the covariances are not set.
- `tf`: the plugin will publish the pose of the link it is attached to using a [`tf2::TransformBroadcaster`](http://wiki.ros.org/tf2). The reference frame id is manually set to `world` (in future versions, it might be interesting to let the user set it using an XML tag).

#### Subscribed topics

- `switch_mode` (type `uav_gazebo_msgs/ControlMode`): publish on this topic to change the control scheme used by the drone. Note that, as soon as the mode is switched, the controller will use the last available commands received for that mode (if no inputs had been received for that mode, they will default to zero). It is thus recommended to send at least one command _before_ performing the switch, to make sure that the drone will not use outdated commands.
- `<control-mode>/command` (types `uav_gazebo_msgs/<ControlMode>Control`): set of topics used to receive control inputs. Each topic uses a specific message type from the `uav_gazebo_msgs` package from this repository.

#### Dynamic reconfiguration

The plugin provides a [dynamic reconfigure server](http://wiki.ros.org/dynamic_reconfigure) to change the gains of the different control layers. The server is started under the child namespace `gains`.

Note that for second order (PD) controllers the parameters that can be tuned via the dynamic reconfigure server are not the proportional and derivative gains directly. Instead, they are the natural frequency and the damping coefficient.

In addition, the utility node `configure_controllers` is provided in case multiple drones are being simulated and the gains have to be adapted for all of them. Given a set of drone namespaces, _e.g._, `drone1`, `drone2`, _etc._, you can connect to all servers by running

```
rosrun uav_gazebo configure_controllers drone1 drone2 ...
```

Such node will start a reconfigure server and forward every request to each drone.

Note that by the default the servers are expected to be located in the namespaces `<drone-name>/gains`. If for any reason you need to change this, two parameters can be loaded in the private namespace of `configure_controllers`: `base_ns` and `servers_ns`. They allow to change the name of the servers to be located as `<base_ns>/<drone-name>/<servers_ns>` (note that `/` characters are added automatically when needed between the three names).

### Usage example

It should be rather easy to configure the plugin. In any case, below there is a (minimal) example of URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="drone">
  <xacro:arg name="drone_name" default="drone"/>
  <xacro:property name="prefix" value="$(arg drone_name)"/>

  <link name="${prefix}">
    <visual>
      <geometry>
        <cylinder length="0.01" radius="0.07"/>
      </geometry>
      <material name="orange">
         <color rgba="0.9 0.9 0.1 1.0"/>
     </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.01" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="${6.5e-4}" iyy="${6.5e-4}" izz="${1.2e-3}" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <gazebo>
    <plugin name="${prefix}_plugin" filename="libuav_gazebo_plugin.so">
      <rosNamespace>${prefix}</rosNamespace>
		</plugin>
  </gazebo>
</robot>
```

Note that it accepts a xacro argument, `drone_name`, so that you can spawn multiple drones by changing their name. By namespacing each plugin in `drone_name`, ROS topics collision is prevented.

## Theoretical background

### Model of the Drone

The class of UAVs considered here is the one of quad-rotors. Their state can
be described using a 12-dimensional state composed of:

- position
- linear velocity (in world frame coordinates)
- attitude (X-Y-Z)
- angular rates

These quantities are denoted as follows:

- **Position Vector**:
```math
  \mathbf{p} = \begin{bmatrix} x & y & z \end{bmatrix}^T
```
```math
  \mathbf{p} = \begin{bmatrix} x & y & z \end{bmatrix}^T
```
- **Velocity Vector**:

```math
  \mathbf{v} = \begin{bmatrix} v_x & v_y & v_z \end{bmatrix}^T
```
```math
  \mathbf{v} = \begin{bmatrix} v_x & v_y & v_z \end{bmatrix}^T
```
- **Orientation Vector**:

```math
  \boldsymbol{\rho} = \begin{bmatrix} \varphi & \theta & \psi \end{bmatrix}^T
```
- **Angular Velocity Vector**:

```math
  \dot{\boldsymbol{\rho}} = \begin{bmatrix} \dot{\varphi} & \dot{\theta} & \dot{\psi} \end{bmatrix}^T
```
The orientation of the drone frame can be obtained as a sequence of three elementary **local** rotations. In particular, we consider here the orientation matrix to be expressed by:

```math
\mathbf{R} = \mathbf{R}_x (\varphi) \mathbf{R}_y (\theta) \mathbf{R}_z (\psi) = \begin{bmatrix}
\cos(\psi) \cos(\theta) & -\sin(\psi) \cos(\theta) & \sin(\theta) \\
\sin(\psi) \cos(\varphi) + \sin(\theta) \sin(\varphi) \cos(\psi) & -\sin(\psi) \sin(\theta) \sin(\varphi) + \cos(\psi) \cos(\varphi) & -\sin(\varphi) \cos(\theta) \\
\sin(\psi) \sin(\varphi) - \sin(\theta) \cos(\psi) \cos(\varphi) & \sin(\psi) \sin(\theta) \cos(\varphi) + \sin(\varphi) \cos(\psi) & \cos(\theta) \cos(\varphi)
\end{bmatrix}
```

With this parameterization, it is possible to link the angular rates to the angular velocity (in the world frame) via the relation:

```math
\boldsymbol{\omega} = \mathrm{\mathbf{D}} (\varphi,\theta) \dot{\boldsymbol{\rho}} =\begin{bmatrix}
1 & 0 & \sin\theta \\
0 & \cos\varphi & -\sin\varphi\cos\theta \\
0 & \sin\varphi & \cos\varphi \cos\theta
\end{bmatrix} \dot{\boldsymbol{\rho}}
```

Furthermore, the angular acceleration in the world frame is given by:

```math
\dot{\boldsymbol{\omega}} = \mathrm{\mathbf{D}} \ddot{\boldsymbol{\rho}} + \dot{\mathrm{\mathbf{D}}} \dot{\boldsymbol{\rho}} \qquad \dot{\mathrm{\mathbf{D}}} = \begin{bmatrix}
0 & 0 & \dot{\theta}\cos\theta \\
0 & -\dot{\varphi}\sin\varphi & -\dot{\varphi}\cos\varphi\cos\theta + \dot{\theta}\sin\varphi\sin\theta \\
0 & \dot{\varphi}\cos\varphi & -\dot{\varphi}\sin\varphi\cos\theta - \dot{\theta}\cos\varphi\sin\theta
\end{bmatrix}
```
```

Finally, the dynamic of the model writes as:

- **Equation 1**:

```math
  \dot{\mathbf{v}} = \frac{1}{m}\mathbf{f} + \mathbf{g} = \frac{1}{m}\begin{bmatrix} \sin\theta \\ -\sin\varphi\cos\theta \\ \cos\varphi\cos\theta \end{bmatrix} f - \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix}
```

- **Equation 2**:

```math
  \boldsymbol{\tau} = \mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega}
```

where the control inputs are:

$f,\,\boldsymbol{\tau}$

_i.e._, the thrust magnitude and the torque (expressed in the world frame). Note that in the dynamic model, the inertia is expressed in world frame coordinates. It can be obtained from the constant body-frame inertia by "rotating" it according to:

$\mathbf{I} = \mathbf{R} \, \mathbf{I}_b \, \mathbf{R}^T$

### Control Schemes

#### Direct control

Not much to say here, thrust and torque are directly sent to move the drone.

#### Thrust & angular velocity

The thrust is forwarded _as is_, while the torque is computed using a proportional controller (optionally with a user-supplied feedforward term). To achieve the control, the angular acceleration is computed as:

```math
\dot{\boldsymbol{\omega}}_c = \dot{\boldsymbol{\omega}}^\star + k_\omega \left( \boldsymbol{\omega}^\star - \boldsymbol{\omega} \right)
```
```math
\dot{\boldsymbol{\omega}}_c = \dot{\boldsymbol{\omega}}^\star + k_\omega \left( \boldsymbol{\omega}^\star - \boldsymbol{\omega} \right)
```

And the torque is then computed via the dynamic model of the drone.

#### Thrust & attitude

The thrust is forwarded _as is_. For the attitude control, a proportional and derivative pseud-control is evaluated:

```math
\ddot{\boldsymbol{\rho}}_c = \ddot{\boldsymbol{\rho}}^\star + k_{d,\rho} \left( \dot{\boldsymbol{\rho}}^\star - \dot{\boldsymbol{\rho}} \right) + k_{p,\rho} \left( \boldsymbol{\rho}^\star - \boldsymbol{\rho} \right)
```
```math
\ddot{\boldsymbol{\rho}}_c = \ddot{\boldsymbol{\rho}}^\star + k_{d,\rho} \left( \dot{\boldsymbol{\rho}}^\star - \dot{\boldsymbol{\rho}} \right) + k_{p,\rho} \left( \boldsymbol{\rho}^\star - \boldsymbol{\rho} \right)
```

The corresponding angular acceleration is then computed, and finally the torque is obtained via the dynamic model.

#### Position & yaw

The goal is now to control the position and the yaw (Z) rotation of the drone. To do this, the idea is to consider the drone as a device able to produce an orientable force. First of all, an acceleration pseudo-control is computed via a proportional and derivative controller:

```math
\dot{\mathrm{\mathbf{v}}}_c = \dot{\mathrm{\mathbf{v}}}^\star + k_{d} \left( \mathrm{\mathbf{v}}^\star - \mathrm{\mathbf{v}} \right) + k_{p} \left( \mathrm{\mathbf{p}}^\star - \mathrm{\mathbf{p}} \right)
```
```math
\dot{\mathrm{\mathbf{v}}}_c = \dot{\mathrm{\mathbf{v}}}^\star + k_{d} \left( \mathrm{\mathbf{v}}^\star - \mathrm{\mathbf{v}} \right) + k_{p} \left( \mathrm{\mathbf{p}}^\star - \mathrm{\mathbf{p}} \right)
```

which could be achieved if the force vector

$\mathbf{f}_c = m \dot{\mathbf{v}}_c - m \mathbf{g}$

was applied to the drone. By comparison with the drone dynamic model, such force can be produced if

```math
```math
\mathbf{f}_c = \begin{bmatrix}
\sin\theta \\
-\sin\varphi\cos\theta \\
\cos\varphi\cos\theta
\end{bmatrix} f
```

Solving for the two angles and the force, one obtains:

```math
f = \Vert \mathbf{f}_c \Vert \quad \varphi^\star = \mathrm{atan2}\left( -f_y, f_z \right) \quad \theta^\star = \mathrm{asin} \left( \frac{f_x}{f} \right)
```

which can be forwarded to the thrust and attitude control described above. Note that the yaw is regulated to the desired value by the attitude controller.

#### Velocity & yaw rate

This control mode consists in nothing but a slightly modified version of the **position and yaw** controller. In this case, one can imagine to set the proportional gain of the position controller to zero (in this way, only the desired velocity will be regulated). Similarly, in the attitude controller one can still regulate the first two angles to the values produced by the velocity controller (to properly orient the thrust direction), while for the yaw regulation it suffices to set the proportional gain to zero (so that only the yaw rate will be taken into account).

## Pure Pursuit Controller Node

### 1. Overview

A ROS node that implements a 3‑D pure‑pursuit algorithm over a spline‑interpolated waypoint path.

- Tracks `/drone/odometry` and publishes velocity + yaw‑rate commands.
- Supports “movement”, “fixed” or “poi” yaw modes.
- Live‑tunable via dynamic_reconfigure.

### 2. Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/dringakn/uav_gazebo.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
catkin build
```

### 3. Parameters

| Name                            | Type   | Default          | Description                      |
| ------------------------------- | ------ | ---------------- | -------------------------------- |
| `~waypoints_file_path`          | string | path.json        | JSON file of 3‑D waypoints       |
| `~trajectory_total_time`        | double | 100.0            | Total spline duration (s)        |
| `~trajectory_resolution`        | int    | 1000             | Number of trajectory samples     |
| `~control_loop_hz`              | double | 50.0             | Control update rate (Hz)         |
| `~look_ahead_distance_min`      | double | 2.0              | Min pure‑pursuit look‑ahead (m)  |
| `~look_ahead_distance_max`      | double | 8.0              | Max look‑ahead (m)               |
| `~look_ahead_distance_factor`   | double | 1.0              | Speed→look‑ahead gain            |
| `~kp_pos`, `~kd_pos`, `~ki_pos` | double | 1.2, 0.0001, 0.0 | Position PID gains               |
| `~kp_yaw`, `~kd_yaw`, `~ki_yaw` | double | 1.0, 0.0001, 0.0 | Yaw PID gains                    |
| `~orientation_mode`             | string | `"movement"`     | `"movement" \| "fixed" \| "poi"` |
| `~fixed_yaw`                    | double | 1.57             | Yaw angle for “fixed” mode       |
| `~poi_x`, `~poi_y`, `~poi_z`    | double | 50.0, 50.0, 30.0 | POI coords for “poi” mode        |
| `~enable_plotting`              | bool   | true             | Publish error & metric streams   |

### 4. Subscribed Topics

- **`/drone/odometry`** (`nav_msgs/Odometry`): current pose & velocity.

### 5. Published Topics

- **`/drone/velocity_yawrate/command`** (`uav_gazebo_msgs/VelocityYawRateControl`)
- **`/pure_pursuit_controller/generated_trajectory`** (`nav_msgs/Path`)
- **`/pure_pursuit_controller/input_waypoints`** (`visualization_msgs/MarkerArray`)
- **`/pure_pursuit_controller/lookahead_distance`**, **`/poi`**, **`/drone_position`** (`Marker` / `PoseStamped`)
- Error & metric streams on `/controller/*` for rqt_plot.

### 6. Launch Example

```xml
<launch>
  <node pkg="uav_gazebo" type="pure_pursuit_controller.py" name="pure_pursuit" output="screen">
    <param name="waypoints_file_path" value="$(find uav_gazebo)/missions/path.json"/>
    <param name="control_loop_hz"        value="100.0"/>
    <param name="orientation_mode"       value="poi"/>
  </node>
</launch>
```

### 7. Visualization & Tuning

- **RViz**: add **Path**, **MarkerArray**, **Marker**, and **Pose** displays on the above topics.
- **rqt_plot**: graph `/controller/error_*`, `/controller/control_commands`.
- **rqt_reconfigure**: live‑adjust PID gains, look‑ahead, spline parameters.

**Tip:**  
Pairing RViz for spatial insight with rqt_plot + rqt_reconfigure gives the fastest feedback loop—ideal for zeroing in on under‑ or over‑shoot.

## Joy Drone Controller Node

### 1. Overview

A ROS node that maps joystick inputs to UAV control commands and mode switches.

- Switch between POSITION_YAW, VELOCITY_YAWRATE, THRUST_ATTITUDE, THRUST_VELOCITY, THRUST_TORQUE, INACTIVE via buttons.
- Reset or teleport the drone in Gazebo with LEFT/RIGHT buttons.
- Launches/restarts the pure‑pursuit controller with the SHARE button.

### 2. Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/dringakn/uav_gazebo.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
catkin build
```

### 3. Joystick Mapping

| Button       | Action                                 |
| ------------ | -------------------------------------- |
| CROSS (0)    | Switch to POSITION_YAW                 |
| CIRCLE (1)   | Switch to VELOCITY_YAWRATE             |
| SQUARE (2)   | Switch to THRUST_ATTITUDE              |
| TRIANGLE (3) | Switch to THRUST_VELOCITY              |
| L1 (4)       | Switch to THRUST_TORQUE                |
| R1 (5)       | INACTIVE / STOP                        |
| LEFT (11)    | Teleport drone to (0,0,6)              |
| RIGHT (12)   | Teleport drone to (0,0,1)              |
| SHARE (8)    | (Re)start pure‑pursuit controller node |

| Axis           | Scales                                    |
| -------------- | ----------------------------------------- |
| LEFT_STICK_X/Y | Position or velocity (×10 or ×1)          |
| RIGHT_STICK_X  | Yaw or attitude (×π)                      |
| RIGHT_STICK_Y  | Altitude / thrust (×10)                   |
| L2 / R2        | Acceleration or attitude rates (×1 or ×π) |

### 4. Subscribed Topic

- **`/joy`** (`sensor_msgs/Joy`)

### 5. Published Topics & Services

- **`/drone/position_yaw/command`** (`PositionYawControl`)
- **`/drone/velocity_yawrate/command`** (`VelocityYawRateControl`)
- **`/drone/thrust_attitude/command`** (`ThrustAttitudeControl`)
- **`/drone/thrust_velocity/command`** (`ThrustVelocityControl`)
- **`/drone/thrust_torque/command`** (`ThrustTorqueControl`)
- **`/gazebo/set_model_state`** (`gazebo_msgs/ModelState`)
- **`/drone/switch_mode`** (`SwitchMode` service)

### 6. Launch Example

```xml
<launch>
  <node pkg="uav_gazebo" type="joy_controller.py" name="joy_drone_controller" output="screen">
    <param name="~deadzone" value="0.05"/>
  </node>
</launch>
```

### 7. Tips

- Use **rqt_graph** to verify topic flow.
- Tune axis dead‑zones in your joystick config for smoother control.
- Ensure the `/drone/switch_mode` service is available before starting.
