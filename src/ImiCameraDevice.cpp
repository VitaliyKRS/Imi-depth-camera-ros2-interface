#include "imidepthcamera/ImiCameraDevice.hpp"
#include <rclcpp/rclcpp.hpp>

ImiCameraDevice::ImiCameraDevice() {}
ImiCameraDevice::~ImiCameraDevice() { closeDevice(); }

void ImiCameraDevice::closeDevice() {
  if (NULL != mImiDevice) {
    imiCamClose(mImiDevice);
    mImiDevice = NULL;
  }
}

ImiCameraHandle ImiCameraDevice::getDeviceHandle() { return mImiDevice; }

bool ImiCameraDevice::openDevice() {

  int ret = imiCamOpen(&mImiDevice);
  if (0 != ret) {
    RCLCPP_WARN(rclcpp::get_logger("ImiCameraDevice"),
                "Open ImiCameraDevice Failed! ret = %d\n", ret);
    closeDevice();
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ImiCameraDevice"), "ImiCameraDevice Opened");
  return true;
}