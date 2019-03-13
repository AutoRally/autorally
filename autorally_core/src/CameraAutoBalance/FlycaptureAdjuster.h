/*
* Software License Agreement (BSD License)
* Copyright (c) 2019, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef AUTORALLY_BALANCE_FLYCAP_AUTOBALANCE_H
#define AUTORALLY_BALANCE_FLYCAP_AUTOBALANCE_H

#include "CameraAdjuster.hpp"
#include <flycapture/FlyCapture2.h>

namespace autorally_core
{

/**
 *  @class FlycaptureAdjuster FlycaptureAdjuster.h
 *  "CameraAutoBalance/FlycaptureAdjuster.h"
 *  @brief Implementation of CameraAdjuster for FlyCapture2
 */
class FlycaptureAdjuster : public CameraAdjuster {
  public:
    FlycaptureAdjuster() = default;

    void Connect() override;
    void SetShutter(double x) override;
    void SetGain(double x) override;

  protected:
    FlyCapture2::Camera cam_;    // PointGrey camera handle
    FlyCapture2::Property prop_; // PointGrey property handle
    FlyCapture2::Error err_;     // PointGrey error handle
};

}  // namespace autorally_core

#endif  // AUTORALLY_BALANCE_FLYCAP_AUTOBALANCE_H
