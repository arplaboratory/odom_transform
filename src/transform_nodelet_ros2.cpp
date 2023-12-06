// C++ composable node of odom transform
// Including all required libraries
#include "transform_nodelet_ros2.h"

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

namespace transform_nodelet_ns {

// Creating constructor
OvtransformNodeletClass::OvtransformNodeletClass(const rclcpp::NodeOptions& transform_node)
    : Node("odom_transform_nodelet", transform_node) {
    onInit();  // Calling the onInit function
}

// Destroying the constructor
OvtransformNodeletClass::~OvtransformNodeletClass() {}

// Defining onInit function
void OvtransformNodeletClass::onInit() {
    // Declaring the yaml file parameter
    transform_config_path = this->declare_parameter("transform_config_path", std::string("Initial Value"));
    if (!this->get_parameter("transform_config_path", transform_config_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get param config_path from server.");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Odom Transform Config path: %s", transform_config_path.c_str());
    }

    // Calling the setup function
    setup(transform_config_path);
}

// Defining setup function
void OvtransformNodeletClass::setup(const std::string transform_config_path) {
    // Creating subscriberthis->get_parameter
    sub_odomimu = this->create_subscription<nav_msgs::msg::Odometry>(
        "odomimu", 1, std::bind(&OvtransformNodeletClass::odomCallback, this, std::placeholders::_1));

    // Creating publishers
    pub_odomworldB0 = this->create_publisher<nav_msgs::msg::Odometry>("odomBinB0_from_transform", 1);

    pub_odomworld = this->create_publisher<nav_msgs::msg::Odometry>("odomBinworld_from_transform", 1);

    // Printing information
    RCLCPP_INFO(this->get_logger(), "[odom_transform] Publishing: %s", pub_odomworldB0->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "[odom_transform] Publishing: %s", pub_odomworld->get_topic_name());

    // Loading the yaml file and the parameters defined in the file
    YAML::Node config = YAML::LoadFile(transform_config_path);

    imu_rate = config["imu_rate"].as<float>();
    odom_rate = config["odom_rate"].as<float>();

    RCLCPP_INFO(this->get_logger(), "[odom_transform] imu_rate: %f", imu_rate);
    RCLCPP_INFO(this->get_logger(), "[odom_transform] odom_rate: %f", odom_rate);

    pub_frequency = 1.0 / odom_rate;
    setupTransformationMatrix(transform_config_path);  // Calling the setupTransformationMatrix function
}

// Defining the setupTransformationMatrix function
void OvtransformNodeletClass::setupTransformationMatrix(const std::string transform_config_path) {
    T_ItoC = Eigen::Matrix4d::Identity();
    T_CtoB = Eigen::Matrix4d::Identity();

    YAML::Node config = YAML::LoadFile(transform_config_path);

    auto T_cam_imu = config["T_cam_imu"];
    auto T_cam_body = config["T_cam_body"];
    auto T_B0_W = config["T_B0_W"];

    RCLCPP_INFO(this->get_logger(), "T_cam_imu:");
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_ItoC(i,j) = T_cam_imu["r" + std::to_string(i)]["c" + std::to_string(j)].as<double>();
            RCLCPP_INFO(this->get_logger(), "  r%d_c%d: %f", i, j,
                        T_cam_imu["r" + std::to_string(i)]["c" + std::to_string(j)].as<double>());
                
        }
    }

    RCLCPP_INFO(this->get_logger(), "T_cam_body:");
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_CtoB(i,j) = T_cam_body["r" + std::to_string(i)]["c" + std::to_string(j)].as<double>();
            RCLCPP_INFO(this->get_logger(), "  r%d_c%d: %f", i, j,
                        T_cam_body["r" + std::to_string(i)]["c" + std::to_string(j)].as<double>());
        }
    }

    RCLCPP_INFO(this->get_logger(), "T_B0_W:");
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_B0toW(i,j) = T_B0_W["r" + std::to_string(i)]["c" + std::to_string(j)].as<double>();
            RCLCPP_INFO(this->get_logger(), "  r%d_c%d: %f", i, j,
                        T_B0_W["r" + std::to_string(i)]["c" + std::to_string(j)].as<double>());
        }
    }

    init_world_with_vicon = config["init_world_with_vicon"].as<bool>();

    // TODO: Check this part with Vicon later
    if (init_world_with_vicon) {
        std::string mav_name;
        mav_name = this->declare_parameter("mav_name", std::string("Value"));
        this->get_parameter("mav_name", mav_name);
        std::string viconOdomWTopic1 = config["viconOdomWTopic_ros2"].as<std::string>();
        std::string viconOdomWTopic = "/" + mav_name + viconOdomWTopic1;
        auto sharedInitBodyOdominW = std::make_shared<nav_msgs::msg::Odometry>();
        nav_msgs::msg::Odometry initBodyOdominW;

        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            viconOdomWTopic, 1, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Message callback
                RCLCPP_INFO(this->get_logger(), "Received Vicon Odometry");
                sharedInitBodyOdominW = msg;
                received_message_ = true;
            });

        timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
            if (!received_message_) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for message.");
            }
        });

        if (sharedInitBodyOdominW != nullptr) {
            initBodyOdominW = *sharedInitBodyOdominW;
            RCLCPP_INFO(this->get_logger(), "Got the init T_BtoW from vicon topic %s", viconOdomWTopic.c_str());
            Eigen::Vector3d initBodyPosinW;
            Eigen::Quaterniond initBodyQuatinW;
            initBodyPosinW << initBodyOdominW.pose.pose.position.x, initBodyOdominW.pose.pose.position.y,
                initBodyOdominW.pose.pose.position.z;
            initBodyQuatinW.w() = initBodyOdominW.pose.pose.orientation.w;
            initBodyQuatinW.x() = initBodyOdominW.pose.pose.orientation.x;
            initBodyQuatinW.y() = initBodyOdominW.pose.pose.orientation.y;
            initBodyQuatinW.z() = initBodyOdominW.pose.pose.orientation.z;
            T_B0toW.block(0, 0, 3, 3) = initBodyQuatinW.toRotationMatrix();
            T_B0toW.block(0, 3, 3, 1) = initBodyPosinW;
            T_WtoB0.block(0, 0, 3, 3) = initBodyQuatinW.toRotationMatrix().transpose();
            T_WtoB0.block(0, 3, 3, 1) = -initBodyQuatinW.toRotationMatrix().transpose() * initBodyPosinW;
            T_MtoB0 = T_WtoB0 * T_MtoW;
        }
        else {
            RCLCPP_INFO(this->get_logger(),
                        "Failed to get init T_BtoW from vicon topic %s, use default T_BtoW by setting "
                        "init_world_with_vicon false",
                        viconOdomWTopic.c_str());
        }
    }
    T_ItoB = T_CtoB * T_ItoC;
    T_BtoI.block(0, 0, 3, 3) = T_ItoB.block(0, 0, 3, 3).transpose();
    T_BtoI.block(0, 3, 3, 1) = -T_ItoB.block(0, 0, 3, 3).transpose() * T_ItoB.block(0, 3, 3, 1);
    T_MtoW = T_B0toW * T_ItoB;
}

// Defining odomCallback function
void OvtransformNodeletClass::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_timestamp = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::duration<double>>(current_timestamp - last_timestamp).count() <=
        pub_frequency) {
        return;
    }
    last_timestamp = current_timestamp;

    nav_msgs::msg::Odometry odomIinM = *msg;
    if (!got_init_tf) {
        T_init_tf(0, 3) = odomIinM.pose.pose.position.x;
        T_init_tf(1, 3) = odomIinM.pose.pose.position.y;
        T_init_tf(2, 3) = odomIinM.pose.pose.position.z;
        Eigen::Matrix<double, 4, 1> q_init_tf;
        q_init_tf << odomIinM.pose.pose.orientation.x, odomIinM.pose.pose.orientation.y,
            odomIinM.pose.pose.orientation.z, odomIinM.pose.pose.orientation.w;
        T_init_tf.block(0, 0, 3, 3) = quat_2_Rot(q_init_tf).transpose();

        std::cout << "T_init_tf: " << T_init_tf << std::endl;
        T_init_tf_inv.block(0, 0, 3, 3) = T_init_tf.block(0, 0, 3, 3).transpose();
        T_init_tf_inv.block(0, 3, 3, 1) = -T_init_tf.block(0, 0, 3, 3).transpose() * T_init_tf.block(0, 3, 3, 1);
        T_MtoW = T_MtoW * T_init_tf;
        std::cout << "T_MtoW: " << T_MtoW << std::endl;
        got_init_tf = true;
    }

    if (!odomBinW) {
        odomBinW = std::make_shared<nav_msgs::msg::Odometry>();
        odomBinW->header.frame_id = "world";
    }
    if (!odomBinB0) {
        odomBinB0 = std::make_shared<nav_msgs::msg::Odometry>();
        odomBinB0->header.frame_id = "odom";
    }
    odomBinW->header.stamp = odomIinM.header.stamp;
    odomBinB0->header.stamp = odomIinM.header.stamp;

    q_GinI_eigen << odomIinM.pose.pose.orientation.x, odomIinM.pose.pose.orientation.y,
        odomIinM.pose.pose.orientation.z, odomIinM.pose.pose.orientation.w;
    ;

    T_ItoM.block(0, 0, 3, 3) = quat_2_Rot(q_GinI_eigen).transpose();  // this is right-handed JPL->right-handed

    T_ItoM(0, 3) = odomIinM.pose.pose.position.x;
    T_ItoM(1, 3) = odomIinM.pose.pose.position.y;
    T_ItoM(2, 3) = odomIinM.pose.pose.position.z;

    // T_ItoB0 = T_MtoB0 * T_ItoM;
    // T_ItoW = T_MtoW * T_ItoM;

    T_ItoM = T_init_tf_inv * T_ItoM;
    T_BtoB0 = T_ItoB * T_ItoM * T_BtoI;

    // Transform the body pose in World frame
    T_BtoW = T_B0toW * T_BtoB0;

    R_BtoB0 = T_BtoB0.block(0, 0, 3, 3);
    R_BtoW = T_BtoW.block(0, 0, 3, 3);
    Eigen::Quaterniond q_BinB0(R_BtoB0);
    Eigen::Quaterniond q_BinW(R_BtoW);

    position_BinW = Eigen::Vector4d(T_BtoW(0, 3), T_BtoW(1, 3), T_BtoW(2, 3), T_BtoW(3, 3));
    position_BinB0 = Eigen::Vector4d(T_BtoB0(0, 3), T_BtoB0(1, 3), T_BtoB0(2, 3), T_BtoB0(3, 3));
    skew_ItoB << 0, -T_ItoB(2, 3), T_ItoB(1, 3), T_ItoB(2, 3), 0, -T_ItoB(0, 3), -T_ItoB(1, 3), T_ItoB(0, 3), 0;
    v_iinIMU = Eigen::Vector3d(odomIinM.twist.twist.linear.x, odomIinM.twist.twist.linear.y,
                               odomIinM.twist.twist.linear.z);
    w_BinB = Eigen::Vector3d(odomIinM.twist.twist.angular.x, odomIinM.twist.twist.angular.y,
                             odomIinM.twist.twist.angular.z);
    w_iinIMU = Eigen::Vector3d(odomIinM.twist.twist.angular.x, odomIinM.twist.twist.angular.y,
                               odomIinM.twist.twist.angular.z);
    v_BinB = -T_ItoB.block(0, 0, 3, 3) * skew_ItoB * w_iinIMU + T_ItoB.block(0, 0, 3, 3) * v_iinIMU;
    Eigen::Quaterniond T_BinB0_from_q;
    Eigen::Quaterniond T_BinW_from_q;
    T_BinB0_from_q.x() = q_BinB0.x();
    T_BinB0_from_q.y() = q_BinB0.y();
    T_BinB0_from_q.z() = q_BinB0.z();
    T_BinB0_from_q.w() = q_BinB0.w();
    T_BinW_from_q.x() = q_BinW.x();
    T_BinW_from_q.y() = q_BinW.y();
    T_BinW_from_q.z() = q_BinW.z();
    T_BinW_from_q.w() = q_BinW.w();

    Eigen::Vector3d v_BinB0 = R_BtoB0 * v_BinB;
    Eigen::Vector3d v_BinW = R_BtoW * v_BinB;

    // The POSE component (orientation and position)
    odomBinW->pose.pose.orientation.x = q_BinW.x();
    odomBinW->pose.pose.orientation.y = q_BinW.y();
    odomBinW->pose.pose.orientation.z = q_BinW.z();
    odomBinW->pose.pose.orientation.w = q_BinW.w();
    odomBinW->pose.pose.position.x = position_BinW(0);
    odomBinW->pose.pose.position.y = position_BinW(1);
    odomBinW->pose.pose.position.z = position_BinW(2);

    odomBinB0->pose.pose.orientation.x = q_BinB0.x();
    odomBinB0->pose.pose.orientation.y = q_BinB0.y();
    odomBinB0->pose.pose.orientation.z = q_BinB0.z();
    odomBinB0->pose.pose.orientation.w = q_BinB0.w();
    odomBinB0->pose.pose.position.x = position_BinB0(0);
    odomBinB0->pose.pose.position.y = position_BinB0(1);
    odomBinB0->pose.pose.position.z = position_BinB0(2);

    // The TWIST component (angular and linear velocities)
    odomBinW->child_frame_id = "body";
    odomBinW->twist.twist.linear.x = v_BinW(0);   // vel in world frame
    odomBinW->twist.twist.linear.y = v_BinW(1);   // vel in world frame
    odomBinW->twist.twist.linear.z = v_BinW(2);   // vel in world frame
    odomBinW->twist.twist.angular.x = w_BinB(0);  // we do not estimate this...
    odomBinW->twist.twist.angular.y = w_BinB(1);  // we do not estimate this...
    odomBinW->twist.twist.angular.z = w_BinB(2);
    ;  // we do not estimate this...

    odomBinB0->child_frame_id = "body";
    odomBinB0->twist.twist.linear.x = v_BinB0(0);  // vel in world frame
    odomBinB0->twist.twist.linear.y = v_BinB0(1);  // vel in world frame
    odomBinB0->twist.twist.linear.z = v_BinB0(2);  // vel in world frame
    odomBinB0->twist.twist.angular.x = w_BinB(0);  // we do not estimate this...
    odomBinB0->twist.twist.angular.y = w_BinB(1);  // we do not estimate this...
    odomBinB0->twist.twist.angular.z = w_BinB(2);  // we do not estimate this...

    odomBinW->pose.covariance = odomIinM.pose.covariance;
    odomBinB0->pose.covariance = odomIinM.pose.covariance;
    // if ( odomBinW.pose.covariance(0) > 0.05){
    //   PRINT_ERROR(RED "Drift detected: pose covariance of x-x is too high %.6f\n", odomBinW.pose.covariance(0));
    // }
    // if ( odomBinW.pose.covariance(7) > 0.05){
    //   PRINT_ERROR(RED "Drift detected: pose covariance of y-y is too high %.6f\n", odomBinW.pose.covariance(7));
    // }
    // if ( odomBinW.pose.covariance(14) > 0.05){
    //   PRINT_ERROR(RED "Drift detected: pose covariance of z-z is too high %.6f\n",
    //   odomBinW.pose.covariance(14));
    // }

    pub_odomworld->publish(*odomBinW);
    pub_odomworldB0->publish(*odomBinB0);
}
}  // namespace transform_nodelet_ns

// Converting it into a composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(transform_nodelet_ns::OvtransformNodeletClass)
