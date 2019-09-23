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

#ifndef AUTORALLY_BALANCE_SPINNAKERADJUSTER_H
#define AUTORALLY_BALANCE_SPINNAKERADJUSTER_H

#include "CameraAdjuster.h"
#include "Spinnaker.h"

namespace autorally_core
{

/**
 *  @class SpinnakerAdjuster SpinnakerAdjuster.h
 *  "CameraAutoBalance/SpinnakerAdjuster.h"
 *  @brief Implementation of CameraAdjuster for Spinnaker
 */
class SpinnakerAdjuster : public CameraAdjuster
{
  public:
    /**
     *  Initialize the system and camera pointers
     */
    SpinnakerAdjuster();

    virtual void connect() override;
    virtual void setShutter(double x) override;
    virtual void setGain(double x) override;

  protected:
    Spinnaker::SystemPtr system_;  // System handle provides access to camera list
    Spinnaker::CameraPtr cam_;  // Handle to camera resources
};

}  // namespace autorally_core

#endif  // AUTORALLY_BALANCE_SPINNAKERADJUSTER_H
