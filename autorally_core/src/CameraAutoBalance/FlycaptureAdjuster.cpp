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

#include "FlycaptureAdjuster.h"

using namespace FlyCapture2;

namespace autorally_core {

void FlycaptureAdjuster::Connect() {
    BusManager busMgr;
    PGRGuid guid;

    auto serial_number = GetSerial();
    busMgr.GetCameraFromSerialNumber(static_cast<unsigned int>(serial_number), &guid);
    err_ = cam_.Connect(&guid);

    prop_.onOff = true;
    prop_.autoManualMode = false;
    prop_.absControl = true;
}

void FlycaptureAdjuster::SetShutter(double x) {
    prop_.type = SHUTTER;
    prop_.absValue = x;
    err_ = cam_.SetProperty(&prop_);
}

void FlycaptureAdjuster::SetGain(double x) {
    prop_.type = GAIN;
    prop_.absValue = x;
    err_ = cam_.SetProperty(&prop_);
}

}  // namespace autorally_core
