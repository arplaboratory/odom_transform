# odom_transform

**Developer: Chenyu Wang, Yang Zhou<br />
Affiliation: [NYU ARPL](https://wp.nyu.edu/arpl/)<br />
Maintainer: Yang Zhou (yangzhou@nyu.edu) Yash Shetye (ys5153@nyu.edu) Jiuhong Xiao (jx1190@nyu.edu)<br />**

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

## calibration tips
- If you use the realsense on chip calibration, it changes --  internal intrinsics and extrinsics parameter stored on chip
since the the parameters are tuned, the depth image quality from the sensor will be better (less hole).
- Do we need to change anything on arpl_autonomy side to use vio? 
NO. the intrinsics and extrinsics parameters are stored in https://github.com/arplaboratory/arpl_autonomy_stack/blob/main/config/race2/default/perception/rs455/camchain-imucam-rs455.yaml
- When and How do you update these parameter?
  - when: if the vio quality really sucks, the temperature is very extreme: (<5 C or > 30C). why? temperature change the curvature of the lens (high-school physics) --> intrinsics of the camera change (mostly fx and fy of intrinsic matrix K). Or you hit the camera really haymaker hard, you should definitely do a calibration.
  - how: you should not rely on realsense on chip calibration, that is lame calibration especially for intrinsic calibration. (okay-ish for extrinsic calibration). you should use kalibr. You should perform stereo camera-imu calibration, that should give you both the extrinsic of each camera with respect to IMU and intrinsics of cameras. It would be good to perform a stereo camera calibration first to obtain a prior of intrinsics and extrinsics of stereo camera before the camrea-imu calibration
  - hey I am a smart and lazy guy (that's why I do phd), WHAT IF we use the realsense on chip calibration to obtain parameters and update that yaml file for vio? NOOO. As I mentioned the on chip calibration only takes a bit time and sample to give you a quick calibration, thus it won't give you good calibration quality. Also, it cannot give you extrinsics between camera and imu.
- Can I run vio without arpl_autonomy? Of course sir/madam! You just need to extract key launch configuration from https://github.com/arplaboratory/arpl_autonomy_stack/blob/main/launch/race_platform/race_autonomy.launch.py and https://github.com/arplaboratory/arpl_autonomy_stack/blob/main/launch/race_platform/perception/openvins_d455.launch.py. You have codex / claude / gemini. It should not be hard.
