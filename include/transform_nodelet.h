#ifndef ovtransform_nodelet_CLASS_SRC_ovtransform_nodelet_CLASS_H_
#define ovtransform_nodelet_CLASS_SRC_ovtransform_nodelet_CLASS_H_
#include <nodelet/nodelet.h>
#include "transform.h"


namespace transform_nodelet_ns
{
class OvtransformNodeletClass : public nodelet::Nodelet
{
public:
    OvtransformNodeletClass();
    ~OvtransformNodeletClass();
    void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg);
    virtual void onInit();
private:
    std::string config_path;
    
	ros::Subscriber sub_imu;

    //std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");
    #if ROS_AVAILABLE == 1
    std::shared_ptr<ROS1Visualizer> viz;
    #elif ROS_AVAILABLE == 2
    std::shared_ptr<ROS2Visualizer> viz;
    #endif
};
} // namespace transform_nodelet_ns

#endif /* ovmsckf_nodelet_CLASS_SRC_ovmsckf_nodelet_CLASS_H_ */
