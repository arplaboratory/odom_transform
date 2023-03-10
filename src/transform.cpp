#include "../include/transform.h"
#include <nodelet/nodelet.h>

using namespace odom_transform;

Transform_calculator::Transform_calculator(std::shared_ptr<ros::NodeHandle>  nodeHandle):
  nh(nodeHandle){}

void Transform_calculator::setup() {
  sub_odomimu = nh->subscribe("/OvmsckfNodeletClass_loader/odomimu", 100, &Transform_calculator::odomCallback, this, ros::TransportHints().tcpNoDelay());
  pub_odomworldB0 = nh->advertise<nav_msgs::Odometry>("odomBinB0_from_transform", 100);
  pub_odomworld = nh->advertise<nav_msgs::Odometry>("odomBinworld_from_transform", 100);
  ROS_INFO("<<<<<<>>>>>>Publishing: %s\n", pub_odomworldB0.getTopic().c_str());
  ROS_INFO("<<<<<<>>>>>>Publishing: %s\n", pub_odomworld.getTopic().c_str());
  setupTransformationMatrix();
  nh->getParam("imu_rate", imu_rate);
  nh->getParam("odom_rate", odom_rate);
  ROS_INFO("<<<<<<>>>>>>Read from yaml: %f\n",imu_rate);
  ROS_INFO("<<<<<<>>>>>>Read from yaml: %f\n", odom_rate);

}

void Transform_calculator::setupTransformationMatrix(){

  Eigen::Matrix4d T_ItoC = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_CtoB = Eigen::Matrix4d::Identity();

  nh->getParam("T_cam_imu/r0/c0", T_ItoC(0,0));
  nh->getParam("T_cam_imu/r0/c1", T_ItoC(0,1));
  nh->getParam("T_cam_imu/r0/c2", T_ItoC(0,2));
  nh->getParam("T_cam_imu/r0/c3", T_ItoC(0,3));
  nh->getParam("T_cam_imu/r1/c0", T_ItoC(1,0));
  nh->getParam("T_cam_imu/r1/c1", T_ItoC(1,1));
  nh->getParam("T_cam_imu/r1/c2", T_ItoC(1,2));
  nh->getParam("T_cam_imu/r1/c3", T_ItoC(1,3));
  nh->getParam("T_cam_imu/r2/c0", T_ItoC(2,0));
  nh->getParam("T_cam_imu/r2/c1", T_ItoC(2,1));
  nh->getParam("T_cam_imu/r2/c2", T_ItoC(2,2));
  nh->getParam("T_cam_imu/r2/c3", T_ItoC(2,3));

  nh->getParam("T_cam_body/r0/c0", T_CtoB(0,0));
  nh->getParam("T_cam_body/r0/c1", T_CtoB(0,1));
  nh->getParam("T_cam_body/r0/c2", T_CtoB(0,2));
  nh->getParam("T_cam_body/r0/c3", T_CtoB(0,3));
  nh->getParam("T_cam_body/r1/c0", T_CtoB(1,0));
  nh->getParam("T_cam_body/r1/c1", T_CtoB(1,1));
  nh->getParam("T_cam_body/r1/c2", T_CtoB(1,2));
  nh->getParam("T_cam_body/r1/c3", T_CtoB(1,3));
  nh->getParam("T_cam_body/r2/c0", T_CtoB(2,0));
  nh->getParam("T_cam_body/r2/c1", T_CtoB(2,1));
  nh->getParam("T_cam_body/r2/c2", T_CtoB(2,2));
  nh->getParam("T_cam_body/r2/c3", T_CtoB(2,3));


  pub_frequency = 1.0/odom_rate;

  // TODO: Check this part with Vicon later  
  bool init_world_with_vicon = false;
  // parser->parse_config("init_world_with_vicon", init_world_with_vicon);
  // if (init_world_with_vicon) {
  //   std::string viconOdomWTopic;
  //   parser->parse_config("viconOdomWTopic", viconOdomWTopic);
  //   boost::shared_ptr<nav_msgs::Odometry const> sharedInitBodyOdominW;
  //   nav_msgs::Odometry initBodyOdominW;
  //   sharedInitBodyOdominW = ros::topic::waitForMessage<nav_msgs::Odometry>(viconOdomWTopic, ros::Duration(5));
  //   if(sharedInitBodyOdominW != nullptr) {
  //     initBodyOdominW = *sharedInitBodyOdominW;
  //     PRINT_INFO("Got the init T_BtoW from vicon topic %s\n", viconOdomWTopic.c_str());
  //     // parse initBodyOdominW
  //     Eigen::Vector3d initBodyPosinW;
  //     Eigen::Quaterniond initBodyQuatinW;
  //     initBodyPosinW << initBodyOdominW.pose.pose.position.x, initBodyOdominW.pose.pose.position.y, initBodyOdominW.pose.pose.position.z;
  //     initBodyQuatinW.w() = initBodyOdominW.pose.pose.orientation.w;
  //     initBodyQuatinW.x() = initBodyOdominW.pose.pose.orientation.x;
  //     initBodyQuatinW.y() = initBodyOdominW.pose.pose.orientation.y;
  //     initBodyQuatinW.z() = initBodyOdominW.pose.pose.orientation.z;
  //     T_B0toW.block(0, 0, 3, 3) = initBodyQuatinW.toRotationMatrix();
  //     T_B0toW.block(0, 3, 3, 1) = initBodyPosinW;
  //     T_WtoB0.block(0, 0, 3, 3) = initBodyQuatinW.toRotationMatrix().transpose();
  //     T_WtoB0.block(0, 3, 3, 1) = -initBodyQuatinW.toRotationMatrix().transpose() * initBodyPosinW;
  //     T_MtoB0 = T_WtoB0 * T_MtoW;
  //   }
  //   else {
  //     PRINT_INFO("Failed to get init T_BtoW from vicon topic %s, use default T_BtoW by setting init_world_with_vicon false\n", viconOdomWTopic.c_str());
  //     exit(1);
  //   }
  // } 
    T_ItoB = T_CtoB *T_ItoC; //* T_correct ;
    T_BtoI.block(0,0,3,3) = T_ItoB.block(0,0,3,3).transpose();
    T_BtoI.block(0,3,3,1) = -T_ItoB.block(0,0,3,3).transpose() * T_ItoB.block(0,3,3,1);
    T_MtoW = T_B0toW * T_ItoB; // T_ItoW at zero timestamp
    // PRINT_INFO("odom_transform T_ItoC");
    // print_tf(T_ItoC);

    // PRINT_INFO("odom_transform T_CtoB");
    // print_tf(T_CtoB);

    // PRINT_INFO("odom_transform T_ItoB");
    // print_tf(T_ItoB);

    // PRINT_INFO("odom_transform T_BtoI");
    // print_tf(T_BtoI);

    // PRINT_INFO("odom_transform T_MtoW");
    // T_MtoW_eigen = print_tf(T_MtoW);
    // PRINT_INFO("odom_transform T_B0toW");
    // print_tf(T_B0toW);
}



void Transform_calculator::odomCallback(const nav_msgs::OdometryPtr& msg_in) {
  nav_msgs::Odometry odomIinM = *msg_in;

  if (!got_init_tf){
   
    //state(0) is the timestamp;
    // Eigen::MatrixXd state_est = state->_imu->value();
    // Eigen::Quaterniond q_init_tf;
    // q_init_tf.x() = odomIinM.pose.pose.orientation.x;
    // q_init_tf.y() = odomIinM.pose.pose.orientation.y;
    // q_init_tf.z() = odomIinM.pose.pose.orientation.z;
    // q_init_tf.w() = odomIinM.pose.pose.orientation.w;
    
    // T_init_tf.block(0,0,3,3) =  q_init_tf.normalized().toRotationMatrix();
    T_init_tf(0, 3) = odomIinM.pose.pose.position.x;
    T_init_tf(1, 3) = odomIinM.pose.pose.position.y;
    T_init_tf(2, 3) = odomIinM.pose.pose.position.z;
    //T_init_tf = T_correct * T_init_tf;
    // PRINT_INFO("T_init_tf\n");
     Eigen::Matrix<double, 4,1> q_init_tf;
    //state(0) is the timestamp;
    // Eigen::MatrixXd state_est = state->_imu->value();
    q_init_tf <<odomIinM.pose.pose.orientation.x, odomIinM.pose.pose.orientation.y, odomIinM.pose.pose.orientation.z, odomIinM.pose.pose.orientation.w;
    T_init_tf.block(0,0,3,3) = quat_2_Rot(q_init_tf).transpose();

    std::cout << T_init_tf << std::endl;
    T_init_tf_inv.block(0,0,3,3) = T_init_tf.block(0,0,3,3).transpose();
    T_init_tf_inv.block(0,3,3,1) = -T_init_tf.block(0,0,3,3).transpose()*T_init_tf.block(0,3,3,1);
    T_MtoW = T_MtoW*T_init_tf;
    std::cout << T_MtoW << std::endl;
    got_init_tf = true;
  }

  nav_msgs::Odometry odomBinW;
  odomBinW.header.stamp = odomIinM.header.stamp;
  odomBinW.header.frame_id = "odom";

  nav_msgs::Odometry odomBinB0 ;
  odomBinB0.header.stamp = odomIinM.header.stamp;
  odomBinB0.header.frame_id = "odom";


    Eigen::Matrix<double, 4,1> q_GinI_eigen;
    q_GinI_eigen << odomIinM.pose.pose.orientation.x, odomIinM.pose.pose.orientation.y, odomIinM.pose.pose.orientation.z, odomIinM.pose.pose.orientation.w;;

    Eigen::Matrix4d T_ItoM = Eigen::Matrix4d::Identity(); // from odomIinM
    T_ItoM.block(0,0,3,3) = quat_2_Rot(q_GinI_eigen).transpose(); // this is right-handed JPL->right-handed

    T_ItoM(0, 3) = odomIinM.pose.pose.position.x;
    T_ItoM(1, 3) = odomIinM.pose.pose.position.y;
    T_ItoM(2, 3) = odomIinM.pose.pose.position.z; 
    
    Eigen::Matrix4d T_ItoB0 = Eigen::Matrix4d::Identity(); // from odomIinM
    Eigen::Matrix4d T_ItoW = Eigen::Matrix4d::Identity(); // from odomIinM
    T_ItoB0 = T_MtoB0*T_ItoM;
    T_ItoW = T_MtoW*T_ItoM;

    // TRANSLATION SOLVED if using JPL convention
    T_ItoM.block(0,3,3,1) = T_init_tf_inv.block(0,0,3,3) * T_ItoM.block(0,3,3,1); 
    T_ItoM.block(0,0,3,3) = T_init_tf_inv.block(0,0,3,3) * T_ItoM.block(0,0,3,3) ; 
    // The TF that is required for flight test
    Eigen::Matrix4d T_BtoB0 = (T_ItoB * T_ItoM * T_BtoI);
    
    // Transform the body pose in World frame
    Eigen::Matrix4d T_BtoW = Eigen::Matrix4d::Identity();

    T_BtoW = T_B0toW * T_BtoB0;

    // ROS_INFO("<<<T_BinW");
    // print_tf(T_BtoW);
    Eigen::Quaterniond  q_BinW;
    Eigen::Matrix3d R_BtoW =  T_BtoW.block(0,0,3,3);
    q_BinW = R_BtoW;
    Eigen::Quaterniond  q_BinB0;
    Eigen::Matrix3d R_BtoB0 =  T_BtoB0.block(0,0,3,3);
    q_BinB0 = R_BtoB0;

    Eigen::Vector4d position_BinW (T_BtoW(0,3), T_BtoW(1,3), T_BtoW(2,3), T_BtoW(3,3));
    Eigen::Vector4d position_BinB0 (T_BtoB0(0,3), T_BtoB0(1,3), T_BtoB0(2,3), T_BtoB0(3,3));
    Eigen::Matrix3d skew_ItoB;
    skew_ItoB << 0,-T_ItoB(2,3), T_ItoB(1,3), T_ItoB(2,3), 0, -T_ItoB(0,3), -T_ItoB(1,3),T_ItoB(0,3), 0;
    Eigen::Vector3d v_iinIMU (odomIinM.twist.twist.linear.x, odomIinM.twist.twist.linear.y, odomIinM.twist.twist.linear.z);
    Eigen::Vector3d w_BinB (odomIinM.twist.twist.angular.x, odomIinM.twist.twist.angular.y, odomIinM.twist.twist.angular.z);
    Eigen::Vector3d w_iinIMU (odomIinM.twist.twist.angular.x, odomIinM.twist.twist.angular.y, odomIinM.twist.twist.angular.z);
    Eigen::Vector3d v_BinB = - T_ItoB.block(0,0,3,3) *skew_ItoB * w_iinIMU + T_ItoB.block(0,0,3,3) * v_iinIMU ;
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

    Eigen::Vector3d v_BinB0 =   T_BinB0_from_q.normalized().toRotationMatrix() * v_BinB ;
    Eigen::Vector3d v_BinW =   T_BinW_from_q.normalized().toRotationMatrix() * v_BinB ;

    // Eigen::Vector3d v_BinB0 = T_MtoB0.block(0,0,3,3)* v_BinB ;

    // The POSE component (orientation and position)
    odomBinW.pose.pose.orientation.x = q_BinW.x();
    odomBinW.pose.pose.orientation.y = q_BinW.y();  
    odomBinW.pose.pose.orientation.z = q_BinW.z();
    odomBinW.pose.pose.orientation.w = q_BinW.w();
    odomBinW.pose.pose.position.x = position_BinW(0); 
    odomBinW.pose.pose.position.y = position_BinW(1); 
    odomBinW.pose.pose.position.z = position_BinW(2);
    
    odomBinB0.pose.pose.orientation.x = q_BinB0.x();
    odomBinB0.pose.pose.orientation.y = q_BinB0.y();
    odomBinB0.pose.pose.orientation.z = q_BinB0.z();
    odomBinB0.pose.pose.orientation.w = q_BinB0.w();
    odomBinB0.pose.pose.position.x = position_BinB0(0); 
    odomBinB0.pose.pose.position.y = position_BinB0(1); 
    odomBinB0.pose.pose.position.z = position_BinB0(2);
    
    // The TWIST component (angular and linear velocities)
    odomBinW.child_frame_id = "body";
    odomBinW.twist.twist.linear.x = v_BinW(0);   // vel in world frame
    odomBinW.twist.twist.linear.y = v_BinW(1);   // vel in world frame
    odomBinW.twist.twist.linear.z = v_BinW(2);   // vel in world frame
    odomBinW.twist.twist.angular.x = w_BinB(0); // we do not estimate this...
    odomBinW.twist.twist.angular.y = w_BinB(1); // we do not estimate this...
    odomBinW.twist.twist.angular.z = w_BinB(2);; // we do not estimate this...

    odomBinB0.child_frame_id = "body";
    odomBinB0.twist.twist.linear.x = v_BinB0(0);   // vel in world frame
    odomBinB0.twist.twist.linear.y = v_BinB0(1);   // vel in world frame
    odomBinB0.twist.twist.linear.z = v_BinB0(2);   // vel in world frame
    odomBinB0.twist.twist.angular.x = w_BinB(0); // we do not estimate this...
    odomBinB0.twist.twist.angular.y = w_BinB(1); // we do not estimate this...
    odomBinB0.twist.twist.angular.z = w_BinB(2); // we do not estimate this...

    odomBinW.pose.covariance = odomIinM.pose.covariance;
    
    // if ( odomBinW.pose.covariance(0) > 0.05){
    //   PRINT_ERROR(RED "Drift detected: pose covariance of x-x is too high %.6f\n",  odomBinW.pose.covariance(0));
    // }
    // if ( odomBinW.pose.covariance(7) > 0.05){
    //   PRINT_ERROR(RED "Drift detected: pose covariance of y-y is too high %.6f\n",  odomBinW.pose.covariance(7));
    // }
    // if ( odomBinW.pose.covariance(14) > 0.05){
    //   PRINT_ERROR(RED "Drift detected: pose covariance of z-z is too high %.6f\n",  odomBinW.pose.covariance(14));
    // }
      
    pub_odomworld.publish(odomBinW);

    odomBinB0.pose.covariance = odomIinM.pose.covariance;
    pub_odomworldB0.publish(odomBinB0);
  
}


// int main(int argc, char** argv) {
//   ros::init(argc, argv, "transform_node");

//   std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");
//   std::string config_path;
//   // nh->param<std::string>("config_path", config_path, config_path);

//   if( !nh->getParam("config_path", config_path) )
//     ROS_ERROR("Failed to get param config_path from server.");
//   ROS_INFO("Config path: %s", config_path.c_str());
//   ROS_INFO("<<<OvtransformNodeletClass Constructor");
//   nh->param<std::string>("config_path", config_path, config_path);
//   ROS_INFO("<<<OvtransformNodeletClass line 247");


//   // parser->set_node_handler(nh);
//   ROS_INFO("<<<Line 29");


//   ROS_INFO("<<<OvtransformNodeletClass line 256");
//   std::string verbosity = "DEBUG";
//   // parser->parse_config("verbosity", verbosity);
//   ROS_INFO("<<<OvtransformNodeletClass Constructor 111");
//   auto trans_cal=Transform_calculator(nh);
//   trans_cal.setup();

//   ros::spin();

//   return 0;
// }

