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
    virtual void onInit();
private:
    std::string config_path;
};
} // namespace transform_nodelet_ns

#endif /* ovmsckf_nodelet_CLASS_SRC_ovmsckf_nodelet_CLASS_H_ */
