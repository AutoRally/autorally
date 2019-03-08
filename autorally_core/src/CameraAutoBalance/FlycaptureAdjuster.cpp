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
