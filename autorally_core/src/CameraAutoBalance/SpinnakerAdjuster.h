#ifndef AUTORALLY_BALANCE_SPINNAKER_CAM_H
#define AUTORALLY_BALANCE_SPINNAKER_CAM_H

#include "CameraAdjuster.hpp"
#include "Spinnaker.h"

namespace autorally_core
{

class SpinnakerAdjuster : public CameraAdjuster {
  public:
    SpinnakerAdjuster();

    void Connect() override;
    void SetShutter(double x) override;
    void SetGain(double x) override;

  protected:
    Spinnaker::SystemPtr system_;
    Spinnaker::CameraPtr cam_;
};

}

#endif  // AUTORALLY_BALANCE_SPINNAKER_CAM_H
