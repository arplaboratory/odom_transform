# odom_transform

**Developer: Chenyu Wang, Yang Zhou<br />
Affiliation: [NYU ARPL](https://wp.nyu.edu/arpl/)<br />
Maintainer: Yang Zhou (yangzhou@nyu.edu)<br />**

## Description
This package transforms the input odometry between two frames, including pose and velocity.
Currently it is designed specifically for transforming openvins odometry from IMU frame to robot center body frame.
In the TF tree, we use /mav_name/odom to represent the robot center frame.

## Publisher / Subscriber
The table below also summarized the publication and subscription scheme.
|Type|Namee|Description|
| --- | --- | --- |
|Subscriber|`odom_imu`|odometry from OpenVINS in IMU frame|
|Publisher|`odomBinB0_from_transform`|odometry in body frame starting from first odom received|
|Publisher|`odomBinworld_from_transform`| odometry in body frame starting from world frame [TODO] |


## Parameters Files

|Name|Description|
|---|---|
|`imu_rate`| OpenVINS odometry rate, which is supposed to be equaled to IMU rate | 
|`odom_rate`| Output odometry rate from the node, need to tune according to platform workload to match 100 Hz |
|`T_cam_imu`| transformation from left camera frame to imu frame |
|`T_cam_body`| transformation from left camera frame to body frame |

## How to run

After running openvins, use
``roslaunch odom_transform openvins.launch``

