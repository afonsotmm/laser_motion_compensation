# Scan Deskew – Motion Distortion Compensation for 2D LiDAR

This ROS package implements motion distortion compensation for 2D LiDAR scans acquired while the robot is moving.

When a LiDAR performs a full scan, points are not captured simultaneously. Instead, they are acquired sequentially while the sensor rotates. If the robot moves during this period, each point is measured from a different robot pose, introducing geometric distortions in the resulting point cloud.

This package compensates for this effect by estimating the sensor motion during the scan using robot odometry and correcting each point individually.

---

# Motion Distortion Problem

Consider a LiDAR performing a full scan with frequency:

$F_{scan}$

The duration of a full scan is therefore:

$$
T_{scan} = \frac{1}{F_{scan}}
$$

If the scan contains $N$ points, the time between two consecutive measurements is approximated by:

$$
\Delta t = \frac{T_{scan}}{N}
$$

The acquisition time of the $i$-th point is therefore:

$$
t_i = i \times \Delta t
$$

During this time, the robot may be moving with:

- linear velocity $v$
- angular velocity $\omega$

which are obtained from the robot odometry.

---

# Robot Motion Model

The robot motion is described by the planar kinematic model:

$$
\dot{x}(t) = v \times \cos(\theta(t))
$$

$$
\dot{y}(t) = v \times \sin(\theta(t))
$$

$$
\dot{\theta}(t) = \omega
$$

Assuming constant angular velocity during a scan, the robot orientation at time $t_i$ is:

$$
\theta_i = \omega \times t_i
$$

---

# Robot Position During the Scan

To determine the robot position when the $i$-th LiDAR point is acquired, the motion model must be integrated.

### Integration of the x component

$$
x_i = \int_{0}^{t_i} v \times \cos(\omega \times t) \, dt
$$

### Integration of the y component

$$
y_i = \int_{0}^{t_i} v \times \sin(\omega \times t) \, dt
$$

Solving these integrals yields:

$$
x_i = \frac{v}{\omega} \times \sin(\omega \times t_i)
$$

$$
y_i = \frac{v}{\omega} \times \left(1 - \cos(\omega \times t_i)\right)
$$

These expressions provide the robot position at the moment the $i$-th LiDAR measurement is acquired.

---

# Special Case: Straight Motion

If the angular velocity is close to zero:

$$
\omega \approx 0
$$

the robot motion can be approximated as straight:

$$
x_i = v \times t_i
$$

$$
y_i = 0
$$

---

# Motion Compensation

To remove motion distortion, each point must be expressed in a common reference frame.  
In this implementation, the reference frame corresponds to the LiDAR pose at the beginning of the scan.

For the $i$-th LiDAR point, the robot position during the scan is given by:

$$
x_i = \frac{v}{\omega} \times \sin(\omega \times t_i)
$$

$$
y_i = \frac{v}{\omega} \times \left(1 - \cos(\omega \times t_i)\right)
$$

and the robot orientation is:

$$
\theta_i = \omega \times t_i
$$

Let the LiDAR point measured at time $t_i$ be:

$$
P_i =
\begin{bmatrix}
x_i^{L} \\
y_i^{L}
\end{bmatrix}
$$

The point expressed in the scan start frame is obtained by rotating the point by the robot orientation and adding the robot displacement during the scan.

---

# Corrected Point

The corrected point is computed as:

$$
P_0 = R(\theta_i) \times P_i +
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}
$$

Expanding the expression:

$$
x_0 = \cos(\theta_i) \times x_i^{L} - \sin(\theta_i) \times y_i^{L} + x_i
$$

$$
y_0 = \sin(\theta_i) \times x_i^{L} + \cos(\theta_i) \times y_i^{L} + y_i
$$

This transformation compensates for the robot motion that occurred between the start of the scan and the acquisition time of each LiDAR point.

---

# Implementation

The algorithm implemented in this package follows the steps below:

1. Receive the distorted LiDAR point cloud.
2. Obtain the robot linear and angular velocities from odometry.
3. Estimate the acquisition time of each point in the scan.
4. Compute the robot pose at that instant.
5. Transform each point to the reference frame corresponding to the beginning of the scan.

---

# Assumptions

The method relies on the following assumptions:

- Linear and angular velocities remain constant during the acquisition of a single scan.
- LiDAR points are acquired uniformly during the scan period.
- Odometry provides sufficiently accurate velocity estimates.

---

# ROS Interface

## Subscribed Topics

| Topic | Type | Description |
|------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Robot odometry providing linear and angular velocities |
| `laser_scan_point_cloud` | `sensor_msgs/PointCloud` | Distorted LiDAR scan |
| `laser_scan_frequency` | `std_msgs/Float32` | LiDAR scan frequency |

---

## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `laser_scan_point_cloud_compensated` | `sensor_msgs/PointCloud` | Motion-compensated point cloud |

---

# License

MIT License

## Credits

Developed by **Electrical and Computer Engineering students at FEUP** for **Robot Factory 4.0** (National Robotics Festival 2026):

* *Afonso Mateus*
* *Christian Geyer*
* *Daniel Silva*
* *Pedro Lopes*

---
