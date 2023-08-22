#ifndef ovtransform_CLASS_SRC_ovtransform_CLASS_H_
#define ovtransform_CLASS_SRC_ovtransform_CLASS_H_

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <chrono>
#include <Eigen/Eigen>

namespace odom_transform{

    class Transform_calculator {
        public:
            Transform_calculator(std::shared_ptr<ros::NodeHandle> nodeHandle);
            void setup();
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_in);
            void setupTransformationMatrix();
            Eigen::Matrix<double, 7, 1> print_tf(Eigen::Matrix4d T);

            Eigen::Matrix4d T_ItoC = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_CtoB = Eigen::Matrix4d::Identity();
            // Transformation between vio local (IMU) frame to vicon world frame
            Eigen::Matrix4d T_MtoW = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_ItoB = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_BtoI = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_B0toW = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_WtoB0 = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_MtoB0 = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_init_tf = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_init_tf_inv = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_ItoM = Eigen::Matrix4d::Identity(); // from odomIinM
            // Eigen::Matrix4d T_ItoB0 = Eigen::Matrix4d::Identity(); // from odomIinM
            // Eigen::Matrix4d T_ItoW = Eigen::Matrix4d::Identity(); // from odomIinM
            Eigen::Matrix4d T_BtoB0 = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_BtoW = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d R_BtoB0 = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d R_BtoW = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d skew_ItoB = Eigen::Matrix3d::Zero();

            Eigen::Matrix<double, 7, 1> T_MtoW_eigen;
            Eigen::Matrix<double, 4,1> q_GinI_eigen;

            Eigen::Vector4d position_BinW = Eigen::Vector4d::Zero();
            Eigen::Vector4d position_BinB0 = Eigen::Vector4d::Zero();
            Eigen::Vector3d v_iinIMU = Eigen::Vector3d::Zero();
            Eigen::Vector3d w_BinB = Eigen::Vector3d::Zero();
            Eigen::Vector3d w_iinIMU = Eigen::Vector3d::Zero();
            Eigen::Vector3d v_BinB = Eigen::Vector3d::Zero();
            std::shared_ptr<ros::NodeHandle> nh;
            ros::Subscriber sub_odomimu;
            ros::Publisher pub_odomworldB0;
            ros::Publisher pub_odomworld;
            nav_msgs::OdometryPtr odomBinW;
            nav_msgs::OdometryPtr odomBinB0;
            bool got_init_tf = false;
            bool init_world_with_vicon = false;
            int skip_count = 0;

            double pub_frequency = 0.0;
            std::chrono::high_resolution_clock::time_point last_timestamp{};
            std::chrono::high_resolution_clock::time_point current_timestamp{};
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
