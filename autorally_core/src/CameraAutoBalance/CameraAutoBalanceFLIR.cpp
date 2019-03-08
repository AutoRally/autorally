#include "CameraAutoBalance.hpp"
#include <pluginlib/class_list_macros.h>

#include "SpinnakerAdjuster.h"

namespace autorally_core {

using CameraAutoBalanceFLIR = CameraAutoBalance<SpinnakerAdjuster>;

}

PLUGINLIB_EXPORT_CLASS(autorally_core::CameraAutoBalanceFLIR, nodelet::Nodelet);
