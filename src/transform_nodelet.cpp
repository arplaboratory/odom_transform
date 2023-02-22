#include "../include/transform_nodelet.h"
#include <pluginlib/class_list_macros.h>
using namespace odom_transform;
namespace transform_nodelet_ns
{
OvtransformNodeletClass::OvtransformNodeletClass()
{
}
OvtransformNodeletClass::~OvtransformNodeletClass()
{
  ROS_INFO("OvtransformNodeletClass Destructor");
}

void OvtransformNodeletClass::onInit()
{

  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>(getMTPrivateNodeHandle());

  if( !nh->getParam("config_path", config_path) )
  ROS_ERROR("Failed to get param config_path from server.");
  ROS_INFO("Config path: %s", config_path.c_str());
  ROS_INFO("<<<OvtransformNodeletClass Constructor");

 
  ROS_INFO("<<<OvtransformNodeletClass Constructor 111");
  auto trans_cal= new Transform_calculator(nh);
  trans_cal->setup();
  
}


} // namespace ovmsckf_nodelet_ns

PLUGINLIB_EXPORT_CLASS(transform_nodelet_ns::OvtransformNodeletClass, nodelet::Nodelet)
