#ifndef ovtransform_CLASS_SRC_ovtransform_CLASS_H_
#define ovtransform_CLASS_SRC_ovtransform_CLASS_H_

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <chrono>
#include <Eigen/Eigen>

namespace odom_transform{

class Transform_calculator {
 	public:
 		Transform_calculator(std::shared_ptr<ros::NodeHandle> nodeHandle);

		void setup();
	private:
		void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_in);
        void setupTransformationMatrix();
        Eigen::Matrix<double, 7, 1> print_tf(Eigen::Matrix4d T);

		
        std::shared_ptr<ros::NodeHandle> nh;
		ros::Subscriber sub_odomimu;
        ros::Publisher pub_odomworldB0;
        ros::Publisher pub_odomworld;
    
        // Transformation between vio local (IMU) frame to vicon world frame
        Eigen::Matrix4d T_MtoW = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_ItoB = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_BtoI = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_B0toW = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_WtoB0 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_MtoB0 = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 7, 1> T_MtoW_eigen;

        bool got_init_tf=false;
        Eigen::Matrix4d T_init_tf = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_init_tf_inv = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d hat_M2B0;

        int skip_count = 0;

        float pub_frequency = 0.0;
	//double last_timestamp = -1;
        float imu_rate = 0;
        float odom_rate = 0;
		
};

inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}

inline Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q) {
  Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
  Eigen::Matrix<double, 3, 3> Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                        2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
  return Rot;
}



}// namespace ovtransform




#endif
