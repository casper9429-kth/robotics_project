# Odometry readme

## Files

1. ``src/``
   1. ``ekf_odom.py``
2. ``ìnclude/``
   1. ``odometry.launch``

ekf_odom.py and odometry_mixied_imu.py do the same thing, but the latter is deprecated.

## How to use
# Include the launch file in your launch file
```
<include file="$(find odometry)/include/odometry.launch" />
```

The node will publish a transform from ``odom`` to ``base_link`` and a ``nav_msgs/Odometry`` message on the ``odom`` topic. 
It will also publish a ``geometry_msgs/PoseWithCovarianceStamped`` message on the ``pose_with_covariance`` topic.
Containing the position and orientation of the robot in the ``odom`` frame and the velocity of the robot in the ``base_link`` frame. 




The node subscribes to the following topics:
1. ``/motor/encoders``
2. ``/imu/data``
3. ``/motor/duty_cycles``
4. ``odom_updater/reset_odom_cov``

1. is used to get v,omega from the encoders
2. is used to get omega from the imu
3. is used to get the control input u
4. is used to reset the covariance matrix of the odometry when the anchor is found
