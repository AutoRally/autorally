/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
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

#ifndef AUTORALLY_ADJUSTABLE_CAM_HPP
#define AUTORALLY_ADJUSTABLE_CAM_HPP

namespace autorally_core
{

/**
 *  @class CameraAdjuster CameraAdjuster.h
 *  "CameraAutoBalance/CameraAdjuster.h"
 *  @brief Abstract base class for setting camera exposure and gain.
 *
 *  This class provides an interface for changing exposure (shutter)
 *  and gain parameters. It can be implemented for multiple camera 
 *  interface libraries (FlyCapture2 and Spinnaker at time of writing)
 */
class CameraAdjuster {
  public:
    /**
     *  One-time setup for adjusting exposure and gain. Grab references
     *  to camera objects, etc.
     */
    virtual void Connect() = 0;

    /**
     *  Set the value of the exposure or shutter speed. The units are
     *  not standardized across implementations.
     *  @param x Exposure/shutter value to set
     */
    virtual void SetShutter(double x) = 0;

    /**
     *  Set the value of the sensor gain. The units are not standardized 
     *  across implementations.
     *  @param x Gain value to set
     */
    virtual void SetGain(double x) = 0;

    /**
     *  Store the serial number of the camera, for use in the other methods.
     */
    void SetSerial(int serial_number) {
        camera_serial_number_ = serial_number;
    }

    int GetSerial() {
        return camera_serial_number_;
    }

  protected:
    int camera_serial_number_;
};

}  // namespace autorally_core

#endif  // AUTORALLY_ADJUSTABLE_CAM_HPP
