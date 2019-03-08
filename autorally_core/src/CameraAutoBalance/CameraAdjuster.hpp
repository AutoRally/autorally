#ifndef AUTORALLY_ADJUSTABLE_CAM_HPP
#define AUTORALLY_ADJUSTABLE_CAM_HPP

namespace autorally_core
{

class CameraAdjuster {
  public:
    virtual void Connect() = 0;
    virtual void SetShutter(double x) = 0;
    virtual void SetGain(double x) = 0;


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
