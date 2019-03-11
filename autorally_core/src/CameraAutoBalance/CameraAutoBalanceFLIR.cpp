#include "CameraAutoBalance.h"
#include <pluginlib/class_list_macros.h>

#include "SpinnakerAdjuster.h"

namespace autorally_core {

class CameraAutoBalanceFLIR : public CameraAutoBalance {
public:
    CameraAutoBalanceFLIR() : CameraAutoBalance(new SpinnakerAdjuster)
    {}
};

}

PLUGINLIB_EXPORT_CLASS(autorally_core::CameraAutoBalanceFLIR, nodelet::Nodelet);
