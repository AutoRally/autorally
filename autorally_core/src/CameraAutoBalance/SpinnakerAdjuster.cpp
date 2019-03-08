#include "SpinnakerAdjuster.h"

#include <chrono>
#include <thread>


namespace autorally_core
{

SpinnakerAdjuster::SpinnakerAdjuster() : system_(Spinnaker::System::GetInstance()), cam_(0)
{}

void SpinnakerAdjuster::Connect() {
    cam_ = system_->GetCameras().GetBySerial(std::to_string(GetSerial()));
}

void SpinnakerAdjuster::SetShutter(double x) {
    if (cam_->IsInitialized()) {
        cam_->ExposureTime.SetValue(x);
    }
}

void SpinnakerAdjuster::SetGain(double x) {
    if (cam_->IsInitialized()) {
        cam_->Gain.SetValue(x);
    }
}

}  // namespace autorally_core
