#include "../include/transform.h"

#include <nodelet/nodelet.h>

using namespace odom_transform;

Transform_calculator::Transform_calculator(std::shared_ptr<ros::NodeHandle> nodeHandle) : nh(nodeHandle) {}

void Transform_calculator::setup() {
    sub_odomimu =
        nh->subscribe("odomimu", 1, &Transform_calculator::odomCallback, this, ros::TransportHints().tcpNoDelay());
    pub_odomworldB0 = nh->advertise<nav_msgs::Odometry>("odomBinB0_from_transform", 1);
    pub_odomworld = nh->advertise<nav_msgs::Odometry>("odomBinworld_from_transform", 1);
    ROS_INFO("[odom_transform] Publishing: %s", pub_odomworldB0.getTopic().c_str());
    ROS_INFO("[odom_transform] Publishing: %s", pub_odomworld.getTopic().c_str());
    if (nh->getParam("imu_rate", imu_rate)) {
        ROS_INFO("[odom_transform] imu_rate: %f", imu_rate);
    }
    else {
        ROS_ERROR("[odom_transform] No imu_rate configured!");
    }
    if (nh->getParam("odom_rate", odom_rate)) {
        ROS_INFO("[odom_transform] odom_rate: %f", odom_rate);
    }
    else {
        ROS_ERROR("[odom_transform] No odom_rate configured!");
    }
    if (nh->getParam("mav_name", mav_name)) {
        ROS_INFO("[odom_transform] mav_name: %s", mav_name.c_str());
    }
    else {
        ROS_ERROR("[odom_transform] No mav_name configured!");
    }
    pub_frequency = 1.0 / odom_rate;
    setupTransformationMatrix();
    if (nh->getParam("publish_tf", publish_tf)) {
        ROS_INFO("[odom_transform] publish_tf: %d", publish_tf);
    }
    else {
        ROS_INFO("[odom_transform] publish_tf: false");
        publish_tf = false;
    }
}

void Transform_calculator::setupTransformationMatrix() {
    T_ItoC = Eigen::Matrix4d::Identity();
    T_CtoB = Eigen::Matrix4d::Identity();

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 4; c++) {
            nh->getParam("T_cam_imu/r" + std::to_string(r) + "/c" + std::to_string(c), T_ItoC(r, c));
            nh->getParam("T_cam_body/r" + std::to_string(r) + "/c" + std::to_string(c), T_CtoB(r, c));
            nh->getParam("T_B0_W/r" + std::to_string(r) + "/c" + std::to_string(c), T_B0toW(r, c));
        }
    }
    nh->getParam("init_world_with_vicon", init_world_with_vicon);

    // TODO: Check this part with Vicon later
    if (init_world_with_vicon) {
        std::string viconOdomWTopic;
        nh->getParam("viconOdomWTopic", viconOdomWTopic);
        boost::shared_ptr<nav_msgs::Odometry const> sharedInitBodyOdominW;
        nav_msgs::Odometry initBodyOdominW;
        sharedInitBodyOdominW = ros::topic::waitForMessage<nav_msgs::Odometry>(viconOdomWTopic, ros::Duration(5));
        if (sharedInitBodyOdominW != nullptr) {
            initBodyOdominW = *sharedInitBodyOdominW;
            ROS_INFO("Got the init T_BtoW from vicon topic %s\n", viconOdomWTopic.c_str());
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
            T_WtoB0.block(0, 0, 3, 3).noalias() = initBodyQuatinW.toRotationMatrix().transpose();
            T_WtoB0.block(0, 3, 3, 1).noalias() = -initBodyQuatinW.toRotationMatrix().transpose() * initBodyPosinW;
            T_MtoB0.noalias() = T_WtoB0 * T_MtoW;
        }
        else {
            ROS_INFO(
                "Failed to get init T_BtoW from vicon topic %s, use default T_BtoW by setting "
                "init_world_with_vicon false\n",
                viconOdomWTopic.c_str());
        }
    }
    T_ItoB.noalias() = T_CtoB * T_ItoC;  //* T_correct ;
    T_BtoI.block(0, 0, 3, 3).noalias() = T_ItoB.block(0, 0, 3, 3).transpose();
    T_BtoI.block(0, 3, 3, 1).noalias() = -T_ItoB.block(0, 0, 3, 3).transpose() * T_ItoB.block(0, 3, 3, 1);
    T_MtoW.noalias() = T_B0toW * T_ItoB;  // T_ItoW at zero timestamp
}

void Transform_calculator::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_in) {
    current_timestamp = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::duration<double>>(current_timestamp - last_timestamp).count() <=
        pub_frequency) {
        return;
    }
    last_timestamp = current_timestamp;
    nav_msgs::Odometry odomIinM = *msg_in;
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
        odomBinW = boost::make_shared<nav_msgs::Odometry>();
        odomBinW->header.frame_id = "world";
    }
    if (!odomBinB0) {
        odomBinB0 = boost::make_shared<nav_msgs::Odometry>();
        odomBinB0->header.frame_id = mav_name + "/odom";
    }
    odomBinW->header.stamp = odomIinM.header.stamp;
    odomBinB0->header.stamp = odomIinM.header.stamp;

    q_GinI_eigen << odomIinM.pose.pose.orientation.x, odomIinM.pose.pose.orientation.y,
        odomIinM.pose.pose.orientation.z, odomIinM.pose.pose.orientation.w;
    ;

    T_ItoM.block(0, 0, 3, 3).noalias() =
        quat_2_Rot(q_GinI_eigen).transpose();  // this is right-handed JPL->right-handed

    T_ItoM(0, 3) = odomIinM.pose.pose.position.x;
    T_ItoM(1, 3) = odomIinM.pose.pose.position.y;
    T_ItoM(2, 3) = odomIinM.pose.pose.position.z;

    // T_ItoB0 = T_MtoB0 * T_ItoM;
    // T_ItoW = T_MtoW * T_ItoM;

    T_ItoM = T_init_tf_inv * T_ItoM;
    T_BtoB0.noalias() = T_ItoB * T_ItoM * T_BtoI;

    // Transform the body pose in World frame
    T_BtoW.noalias() = T_B0toW * T_BtoB0;

    R_BtoB0 = T_BtoB0.block(0, 0, 3, 3);
    R_BtoW = T_BtoW.block(0, 0, 3, 3);
    Eigen::Quaterniond q_BinB0(R_BtoB0);
    Eigen::Quaterniond q_BinW(R_BtoW);

    position_BinW = Eigen::Vector4d(T_BtoW(0, 3), T_BtoW(1, 3), T_BtoW(2, 3), T_BtoW(3, 3));
    position_BinB0 = Eigen::Vector4d(T_BtoB0(0, 3), T_BtoB0(1, 3), T_BtoB0(2, 3), T_BtoB0(3, 3));
    skew_ItoB << 0, -T_ItoB(2, 3), T_ItoB(1, 3), T_ItoB(2, 3), 0, -T_ItoB(0, 3), -T_ItoB(1, 3), T_ItoB(0, 3), 0;
    v_iinIMU.noalias() = Eigen::Vector3d(odomIinM.twist.twist.linear.x, odomIinM.twist.twist.linear.y,
                                         odomIinM.twist.twist.linear.z);
    w_BinB.noalias() = Eigen::Vector3d(odomIinM.twist.twist.angular.x, odomIinM.twist.twist.angular.y,
                                       odomIinM.twist.twist.angular.z);
    w_iinIMU.noalias() = Eigen::Vector3d(odomIinM.twist.twist.angular.x, odomIinM.twist.twist.angular.y,
                                         odomIinM.twist.twist.angular.z);
    v_BinB.noalias() = -T_ItoB.block(0, 0, 3, 3) * skew_ItoB * w_iinIMU + T_ItoB.block(0, 0, 3, 3) * v_iinIMU;

    v_BinB0.noalias() = R_BtoB0 * v_BinB;
    v_BinW.noalias() = R_BtoW * v_BinB;

    if (pub_odomworld.getNumSubscribers() != 0) {
        odomBinW->pose.pose.orientation.x = q_BinW.x();
        odomBinW->pose.pose.orientation.y = q_BinW.y();
        odomBinW->pose.pose.orientation.z = q_BinW.z();
        odomBinW->pose.pose.orientation.w = q_BinW.w();
        odomBinW->pose.pose.position.x = position_BinW(0);
        odomBinW->pose.pose.position.y = position_BinW(1);
        odomBinW->pose.pose.position.z = position_BinW(2);

        odomBinW->child_frame_id = mav_name + "/body";
        odomBinW->twist.twist.linear.x = v_BinW(0);   // vel in world frame
        odomBinW->twist.twist.linear.y = v_BinW(1);   // vel in world frame
        odomBinW->twist.twist.linear.z = v_BinW(2);   // vel in world frame
        odomBinW->twist.twist.angular.x = w_BinB(0);  // we do not estimate this...
        odomBinW->twist.twist.angular.y = w_BinB(1);  // we do not estimate this...
        odomBinW->twist.twist.angular.z = w_BinB(2);
        ;  // we do not estimate this...
        odomBinW->pose.covariance = odomIinM.pose.covariance;

        pub_odomworld.publish(odomBinW);

        if (publish_tf) {
            tf::StampedTransform trans = get_stamped_transform_from_odom(odomBinW);
            trans.frame_id_ = "world";
            trans.child_frame_id_ = mav_name + "/body";
            mTfBr.sendTransform(trans);
        }

        // if ( odomBinW.pose.covariance(0) > 0.05){
        //   PRINT_ERROR(RED "Drift detected: pose covariance of x-x is too high %.6f\n",
        //   odomBinW.pose.covariance(0));
        // }
        // if ( odomBinW.pose.covariance(7) > 0.05){
        //   PRINT_ERROR(RED "Drift detected: pose covariance of y-y is too high %.6f\n",
        //   odomBinW.pose.covariance(7));
        // }
        // if ( odomBinW.pose.covariance(14) > 0.05){
        //   PRINT_ERROR(RED "Drift detected: pose covariance of z-z is too high %.6f\n",
        //   odomBinW.pose.covariance(14));
        // }
    }
    if (pub_odomworldB0.getNumSubscribers() != 0) {
        odomBinB0->pose.pose.orientation.x = q_BinB0.x();
        odomBinB0->pose.pose.orientation.y = q_BinB0.y();
        odomBinB0->pose.pose.orientation.z = q_BinB0.z();
        odomBinB0->pose.pose.orientation.w = q_BinB0.w();
        odomBinB0->pose.pose.position.x = position_BinB0(0);
        odomBinB0->pose.pose.position.y = position_BinB0(1);
        odomBinB0->pose.pose.position.z = position_BinB0(2);

        odomBinB0->child_frame_id = mav_name + "/body";
        odomBinB0->twist.twist.linear.x = v_BinB0(0);  // vel in world frame
        odomBinB0->twist.twist.linear.y = v_BinB0(1);  // vel in world frame
        odomBinB0->twist.twist.linear.z = v_BinB0(2);  // vel in world frame
        odomBinB0->twist.twist.angular.x = w_BinB(0);  // we do not estimate this...
        odomBinB0->twist.twist.angular.y = w_BinB(1);  // we do not estimate this...
        odomBinB0->twist.twist.angular.z = w_BinB(2);  // we do not estimate this...
        odomBinB0->pose.covariance = odomIinM.pose.covariance;

        pub_odomworldB0.publish(odomBinB0);
        if (publish_tf) {
            tf::StampedTransform trans = get_stamped_transform_from_odom(odomBinB0);
            trans.frame_id_ = mav_name + "/odom";
            trans.child_frame_id_ = mav_name + "/body";
            mTfBr.sendTransform(trans);
        }
    }
}
