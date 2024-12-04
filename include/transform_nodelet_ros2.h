// C++ header file for the transform_nodelet_ros2 composable node
// Including all required libraries
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <Eigen/Eigen>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.h"


namespace transform_nodelet_ns
{
    // Creating a class and declaring all required functions and defining variables
    class OvtransformNodeletClass : public rclcpp::Node
    {
        public:
            OvtransformNodeletClass(const rclcpp::NodeOptions &transform_node);
            ~OvtransformNodeletClass();
            void onInit();
        private:
	    struct RateController {
		    double target_period;
		    double token_deficit;
		    rclcpp::Time last_update;
		    bool first_msg;
		    RateController (double rate)
			    : target_period(0.95/rate)
			    , token_deficit(0.0)
		            , first_msg(false) {}

		    bool checkAndUpdate(const rclcpp::Time& current_time) {
			    if (!first_msg) {
				    last_update = current_time;
				    first_msg = true;
				    return true;
			    }

			    double elapsed = (current_time - last_update).seconds();
			    token_deficit += elapsed - target_period;

			    token_deficit = std::min(target_period, std::max(-target_period, token_deficit));

			    if (token_deficit >= 0.0) {
				    last_update = current_time;
				    return true;
			    }
			    return false;
		    }
	    };
	    std::unique_ptr<RateController> rate_controller_;
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void setup(const std::string filePath);
            void setupTransformationMatrix(const std::string filePath);
            std::string transform_config_path;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
            rclcpp::TimerBase::SharedPtr timer_;
            bool received_message_ = false;
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
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odomimu;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odomworldB0;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odomworld;
            bool got_init_tf = false;
            bool init_world_with_vicon = false;
            bool publish_tf = true;
	    bool bPublishBinB0 = false;
	    bool bPublishBinW = false;
            int skip_count = 0;
            std::string mav_name;
	     
            tf2_ros::TransformBroadcaster mTfBr; //tf object responsible for publishing frames between world and body or odom and body
            double pub_frequency = 0.0;
            float imu_rate = 0;
            float odom_rate = 0;

    };
    // This function creates the tf message to be published. Takes input from odom and outputs a msg of type geometry_msgs::msg::TransformStamped 
    inline geometry_msgs::msg::TransformStamped get_stamped_transform_from_odom(nav_msgs::msg::Odometry &odom) {
            auto q_ItoC = odom.pose.pose.orientation;
            auto p_CinI = odom.pose.pose.position;
            geometry_msgs::msg::TransformStamped trans;
            trans.header.stamp = odom.header.stamp;
            //geometry_msgs::msg::Quaternion quat = q_ItoC;//(q_ItoC.x, q_ItoC.y, q_ItoC.z, q_ItoC.w)
            trans.transform.rotation = q_ItoC;
            geometry_msgs::msg::Vector3 orig; //(p_CinI.x, p_CinI.y, p_CinI.z)
            orig.x = p_CinI.x;
            orig.y = p_CinI.y;
            orig.z = p_CinI.z;
            trans.transform.translation = orig;
            return trans;
    }

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
} // namespace transform_nodelet_ns
