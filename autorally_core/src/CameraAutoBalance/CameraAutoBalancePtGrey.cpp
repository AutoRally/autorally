#include "CameraAutoBalance.hpp"
#include <pluginlib/class_list_macros.h>

#include "FlycaptureAdjuster.h"

namespace autorally_core {

using CameraAutoBalancePtGrey = CameraAutoBalance<FlycaptureAdjuster>;

}

PLUGINLIB_EXPORT_CLASS(autorally_core::CameraAutoBalancePtGrey, nodelet::Nodelet);
