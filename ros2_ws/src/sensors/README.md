# Sensor Data Processing

The **sensors** package handles low-level processing and conversion of raw sensor outputs into ROS-standardized messages for state estimation. It currently supports the **IMU**, **depth sensor**, and **DVL**, performing tasks such as:

- Coordinate-frame transformations  
- Kinematic conversions  
- Validity flagging and message synchronization  

These processed sensor streams are used downstream by the **EKF**, **controller**, and **mapping/localization** pipelines.

---

## Table of Contents
- [Sensor Data Processing](#sensor-data-processing)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [IMU Processing](#imu-processing)
    - [**1. Gravity corrected acceleration**](#1-gravity-corrected-acceleration)
    - [**2. Gyroscope angular rates**](#2-gyroscope-angular-rates)
    - [**3. Orientation**](#3-orientation)
  - [Depth Sensor Processing](#depth-sensor-processing)
  - [DVL Processing](#dvl-processing)
    - [**1.Transformation Logic**](#1transformation-logic)
  - [Usage](#usage)
  - [Nodes](#nodes)
    - [Published Topics](#published-topics)
    - [Subscribed Topics](#subscribed-topics)
  - [Installation](#installation)
    - [Dependencies](#dependencies)
    - [Building](#building)
    - [Running](#running)
- [License](#license)

---

## Overview

This package receives raw sensor messages and transforms them into normalized, physically meaningful quantities for the AUV system. Each sensor pipeline follows a similar pattern:

1. **Parse raw measurements** from hardware drivers  
2. **Apply calibration** (Global ENU → Pool calibrations)  
3. **Perform coordinate-frame transformations** from sensor frame → vehicle (body) frame  
4. **Publish cleaned, validated ROS messages** for use by the state estimator  

Different sensors follow different mathematical models (detailed below). The vehicle frame is located at the AUV's center of mass, its orientation is identical to the coordiante axes system used in the propulsion package (shown in the propulsion README). The pool frame is located at the AUV's starting location in the pool and is also oriented +X forward, +Y left, +Z up. The pool inertial frame is denoted as $i$, the vehicle frame is denoted as $v$. 

---

## IMU Processing

The IMU produces 3-axis **accelerometer**, **gyroscope**, and **magnetometer** readings. These are converted into body-frame free acceleration, angular velocity, and orientation estimates.

### **1. Gravity corrected acceleration**

Raw accelerometer readings are the **specific force** in the sensor frame *s*: 

$$
f_s = C_{sv} (  C_{vi} ( \ddot{r}_i^{vi} - g_i))
$$

Where:

$C_{sv}$: Rotation matrix from vehicle to sensor frame 

$C_{vi}$: Rotation matrix from inertial (pool) to vehicle frame

$\ddot{r}_i^{vi}$: Free acceleration of the vehicle expressed in the inertial frame

$g_i$: Gravity vector expressed in inertial frame. Always $g_i = [0 \, 0 \, -9.81]^{\mathsf{T}}$

Note that the above relation assumes a sufficiently small offset between the sensor and vehicle frames, $r_v^sv$. Otherwise, the relation would include more terms. Multiplying by $C_{sv}^T$ on both sides and isolating for the free acceleration:

$$
\ddot{r}_v^{vi} = C_{sv}^T f_s + C_{vi} g_i = C_{vs} f_s + C_{vi} g_i
$$

$C_{sv}$ can be directly obtained from the IMU's physcial mounting direction on the AUV. For $C_{vi}$:

$$
 C_{vi} =  C_{vs} C_{si} 
$$

Finally, 

$$
\ddot{r}_v^{vi} = C_{vs} ( f_s + C_{si} g_i ) 
$$

$C_{si}$ can be obtained from the quaternion orientation $q_{si}$ calculated our IMU messages (see section 3):

$$
C_{si}(q_{si}) = \begin{bmatrix}
  2(w^2 + x^2) - 1 & 2(xy - wz) & 2(xz + wy) \\
  2(xy + wz) & 2(w^2 + y^2) - 1 & 2(yz - wx) \\
  2(xz - wy) & 2(yz + wx) & 2(w^2 + z^2) - 1
\end{bmatrix}
$$


### **2. Gyroscope angular rates**
The gyro sensor model is simpler than the accelerometers, because the angular velocity of all points on a rigid body is the same. Essentially, the measured angular rates, $\omega$, are the body rates of the vehicle, expressed in the sensor frame:

$$
\omega = \omega_s^{vi} = C_{sv} \omega_v^{vi}
$$

Isolatiing for $\omega_v^{vi}$:

$$
\omega_v^{vi} = C_{sv}^T \omega_s
$$


### **3. Orientation**

Absolute orientation is given to us from an Earth-fixed navigation frame (TBD if  magnetic ENU) , $n$, to the IMU as a quaternion from the sensor frame , $s$, as $q_{sn}$. However, we wish to have our orientation expressed in the pool's inertial $i$ frame because it is more intuitive. By taking an initial measurement of the pool's frame orientation in the $n$ frame, we have $q_{in}$. Thus,

$$
q_{si} = q_{sn} \otimes q_{in}^{-1}
$$

---

## Depth Sensor Processing

The depth sensor gives us the z-positon of the *depth sensor* in the pool's inertial frame, expressed in the pool's frame. This is denoted as:

$$
[r_i^{si}]_z
$$

 Keep in mind that the letter *s* is used to denote the frame of the sensor in question. The *s* here symoblizes a different frame than the one in the section above. 
 
 Our goal is to use this reading to calculate the z-position of the vehicle frame, $[r_i^{vi}]_z$. They are related as follows:

 $$
[r_i^{vi}]_z = [r_i^{si}]_z + [r_i^{vs}]_{z}
 $$
 
 Where $[r_i^{vs}]_{z}$ is the translation vector from the sensor frame to the vehicle frame, expressed in the inertial frame. 
 
From the depth sensor's mounting on the AUV, we inheretnly know $r_v^{vs}$. Since the pool's reference frame may not necessarily be aligned with the vehicle's reference frame, we must use a rotation matrix to re-express $r_v^{vs}$ in the inertial frame:

$$
r_v^{vs} = C_{iv} r_i^{vs}
$$

$C_{iv}$: Rotation matrix from vehicle to inertial (pool) frame

$$
C_{iv}(q_{iv}) = \begin{bmatrix}
  2(w^2 + x^2) - 1 & 2(xy - wz) & 2(xz + wy) \\
  2(xy + wz) & 2(w^2 + y^2) - 1 & 2(yz - wx) \\
  2(xz - wy) & 2(yz + wx) & 2(w^2 + z^2) - 1
\end{bmatrix}
$$

Note that $C_{iv}$ need not be calculated from the equation above. From our IMU data, we have:

$$
 C_{vi} =  C_{vs} C_{si} 
$$

Since $C_{iv} = C_{vi}^T$, we can easily get $C_{iv}$ without re-calculating from the quaternion.

Finally,

$$
[r_i^{vi}]_z = [r_i^{si}]_z + [C_{iv} r_v^{vs}]_{z} = [r_i^{si}]_z + [C_{vi}^T r_v^{vs}]_{z}
$$

Where:

$[r_i^{vi}]_z$: z-position of the vehicle frame, expressed in the pool inertial frame. 

$[r_i^{si}]_z$: z-position of the depth sensor frame, expressed in the pool's inertial frame

$r_v^{vs}$: the vector from the sensor to the vehicle frame, expressed in the vehicle's frame of reference (obtained from our CAD). 

---


## DVL Processing


The DVL provides position data for the sensor's location relative to the pool frame. It utilizes an internal Kalman Filter to fuse IMU and acoustic velocity measurements for dead-reckoning position estimation. All relevant documentation about the DVL a50 can be [here](https://docs.waterlinked.com/dvl/dvl-a50/). 

As mentioned above, we operate in three coordinate reference frames:
- **Pool Inertial Frame ($p$):** Origin located at the AUV's starting point. 
- **DVL Inertial Frame ($i2$):** Origin located the DVL's starting point.
- **Vehicle Frame ($v$):** origin at the AUV's **Center of Mass (CM)**.
- **DVL Body Frame ($d$):** origin at the DVL's physical mounting point.

Because the DVL output represents the DVL's location ($d$) and not the AUV's Center of Mass ($v$), the raw coordinates must be transformed into the pool frame before adding the offset between the DVL inertial frame($i2$) and the pool inertial frame ($p$).

### **1. Position Transformation Logic**

To calculate the AUV's position in the $p$ frame, we need to go from the $p$ frame to the $i2$ frame, then from the $i2$ frame to the $d$ frame, then from the $d$ frame to the $v$ frame:

$$
r_p^{vp} = r_p^{i2p} + r_{p}^{di2} + r_p^{vd}
$$

Since our DVL gives us $r_{i2}^{di2}$ and our CAD gives us $r_v^{vd}$, we re-express the above equation 
as:  

$$
r_p^{vp} = r_p^{i2p} + C_{pi2}  r_{i2}^{di2} + C_{pv}  r_v^{vd}
$$

Let's know figure out how we can get the three remaining variables in the equation. 

- $r_p^{i2p}$ is equal to $r_v^{dv}$. This is because the vector from the AUV's starting point to the DVL's starting point
is simply the vector from the vehicle frame to the DVL frame at startup. 

- Based on the coordinate system adopted by our DVL, we also can calcaulate the
rotation matrix from the DVL's inertial frame to the pool inertial frame, giving us $C_{pi2}$. 

 - Finally, $C_{pv}$ can be obtained from our IMU readings. 

As for calculating the AUV's velocity from the DVL's velocity measurements, it invloves more complex calculations
involving the orientation and angular velocity of the AUV. It is currently not supported. 

### **2. Velocity Transformation Logic**
The DVL provides velocity readings of the DVL in the the DVL's body frame. We want to convert them into velocity
measurements of the AUV in the inertial pool frame. 

From 3D-kinematics:

$$
v^b = v^a + \omega \times r^{ba}_i + (v^{ba})_{xyz}
$$

Where:
$v_b$ = velocity of $b$ in the inertial frame of reference

$v_a$ = velocity of $a$ in the inertial frame of reference

$v^{ba}$ = velocity of “$b$ with respect to $a$” as measured by an
observer attached to the rotating x, y, z frame of reference that is mounted on $a$

$\omega$ = angular velocity of the x, y, z frame of reference, in the inertial frame of reference

$r^{ba}_i$ = postion of “$b$ with respect to $a$”, expressed in the inertial frame

Now let us apply this equation on the AUV and DVL, with the pool frame being the "inertial frame" and the AUV vehicle frame being the "rotating x, y, z frame".

$$
v_p^{dvl}= v_p^{auv} + \omega \times r^{dvl,auv}_p + (v^{dvl,auv})_{v}
$$

Since the dvl is physically fixed on the auv, $(v^{dvl,auv})_{v} = 0$. The DVL gives us $v^{dvl}$, so we can isolate and solve for $v^{auv}$

$$
v_p^{auv}= v_p^{dvl} - \omega_p \times r^{dvl,auv}_p 
$$

Finally rewriting the above values in the frames they are "given" in:

$$
v_p^{auv}= C_{pv}  C_{v,dvl}  v_{dvl}^{dvl} - C_{pv}(\omega_v \times r^{dvl,auv}_v) 
$$


A good exercise would be re-applying the equation at the beginning of this section and take $b$ to be $auv$ and $a$ to to be $dvl$. You should arrive to the same final expression for $v^{auv}$. 

## Usage

This package is not used directly by operators. It runs alongside the sensor drivers and publishes processed data for the estimation stack.

---

## Nodes

The package provides a single ROS node: `sensor_node`.

- Input: subscribes to all raw sensor data topics. TBD on how DVL data will be read

- Outputs: publishes all processed data to their respective topics

---

### Published Topics

| Topic | Message | Description |
|-------|---------|-------------|
| `auv_frame/imu` | Imu | Processed imu data in `auv` frame. Free acceleration |
| `auv_frame/depth` | Float64 | AUV's depth in `pool` frame |
| `/sensors/dvl` | TBD | TBD |
| `state/pose` | PoseStamped | Aggregated pose of the AUV in `pool` frame | 

---

### Subscribed Topics

| Topic | Message | Description |
|-------|---------|-------------|
| `imu/data` | Vendor IMU message | Data in imu's body frame. Specific force  |
| `/sensors/depth/z` | Pressure sensor message | Depth of sensor probe |
| `/raw/dvl` | Vendor DVL message | TBD |

---

## Installation

### Dependencies

- Eigenv3
- rclcpp
- std_msg
- sensor_msgs
- geometry_msgs
- message_filters

---

### Building

```bash
source /opt/ros/humble/setup.bash
cd <AUV-2026>/ros2_ws
colcon build --packages-select sensors
```

---

### Running

Launch all sensor data processors

    ros2 launch sensors sensor_node.launch.py

---

# License

Released under GPLv3

---
