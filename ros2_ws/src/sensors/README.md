# Sensor Data Processing

The **sensors** package handles low-level processing and conversion of raw sensor outputs into ROS-standardized messages for state estimation. It currently supports the **IMU**, **depth sensor**, and **DVL**, performing tasks such as:

- Coordinate-frame transformations  
- Kinematic conversions  
- Validity flagging and message synchronization  

These processed sensor streams are used downstream by the **EKF**, **controller**, and **mapping/localization** pipelines.

---

## Table of Contents
- [Overview](#overview)
- [IMU Processing](#imu-processing)
- [Depth Sensor Processing](#depth-sensor-processing)
- [DVL Processing](#dvl-processing)
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
3. **Perform coordinate-frame transformations** from sensor frame → body frame  
4. **Publish cleaned, validated ROS messages** for use by the state estimator  

Different sensors follow different mathematical models (detailed below).

---

## IMU Processing

The IMU produces 3-axis **accelerometer**, **gyroscope**, and **magnetometer** readings. These are converted into body-frame free acceleration, angular velocity, and orientation estimates.

### **1. Gravity corrected accelerometion**

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
The gyro sensor model is simpler than the accelerometers. Essentially, the measured angular rates, $\omega$ , are the
body rates of the vehicle, expressed in the sensor frame:

$$
\omega_s = C_{sv} \omega_v^{vi}
$$


### **3. Orientation**

Orientation is given to us from an Earth-fixed navigation frame (TBD if  magnetic ENU) , $n$, to the IMU as a quaternion from the sensor frame , $s$, as $q_{sn}$. However, we wish to have our orientation expressed in the pool's inertial $i$ frame because it is more intuitive. By taking an initial measurement of the pool's frame orientation in the $n$ frame, we have $q_{in}$. Thus,

$$
q_{si} = q_{sn} \otimes q_{in}^{-1}
$$

---

## Depth Sensor Processing

TBD

---

## DVL Processing

TBD

---

## Usage

This package is not used directly by operators. It runs alongside the sensor drivers and publishes processed data for the estimation stack.


---

## Nodes

TBD

---

### Published Topics

| Topic | Message | Description |
|-------|---------|-------------|
| `/sensors/imu` | TBD | TBD |
| `/sensors/depth` | TBD` | TBD |
| `/sensors/dvl` | TBD | TBD |

---

### Subscribed Topics

| Topic | Message | Description |
|-------|---------|-------------|
| `/raw/imu` | Vendor IMU message | TBD |
| `/raw/depth` | Pressure sensor message | TBD |
| `/raw/dvl` | Vendor DVL message | TBD |

---

## Installation

### Dependencies

- Eigenv3

---

### Building

```bash
source /opt/ros/humble/setup.bash
cd <AUV-2026>/ros2_ws
colcon build --symlink-install
```

---

### Running

TBD

---

# License

Released under GPLv3

---
