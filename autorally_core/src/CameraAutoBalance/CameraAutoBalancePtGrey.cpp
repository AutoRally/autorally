#include "CameraAutoBalance.h"
#include <pluginlib/class_list_macros.h>

#include "FlycaptureAdjuster.h"

namespace autorally_core {

class CameraAutoBalancePtGrey : public CameraAutoBalance {
public:
    CameraAutoBalancePtGrey() : CameraAutoBalance(new FlycaptureAdjuster)
    {}
};

}

PLUGINLIB_EXPORT_CLASS(autorally_core::CameraAutoBalancePtGrey, nodelet::Nodelet);
