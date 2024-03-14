#include "imidepthcamera/ImiNectDevice.hpp"
#include <rclcpp/rclcpp.hpp>

ImiNectDevice::ImiNectDevice() {}
ImiNectDevice::~ImiNectDevice() { closeDevice(); }

void ImiNectDevice::closeDevice() {
  if (NULL != mImiDevice) {
    imiCloseDevice(mImiDevice);
    mImiDevice = NULL;
  }

  if (NULL != mDeviceAttr) {
    imiReleaseDeviceList(&mDeviceAttr);
    mDeviceAttr = NULL;
  }
  imiDestroy();
}

ImiCameraIntrinsic ImiNectDevice::getCameraIntrinsic() {
  ImiCameraIntrinsic intrinsic;
  ImiBOOL status;
  uint32_t size = sizeof(ImiBOOL);
  int ret = imiGetDeviceProperty(
      mImiDevice, IMI_PROPERTY_COLOR_INTRINSIC_PARAMS, &intrinsic, &size);

  if (0 != ret) {
    RCLCPP_WARN(rclcpp::get_logger("ImiNectDevice"),
                "ImiNectDevice property get failed! ret = %d", ret);
  }

  return intrinsic;
}
ImiDeviceHandle ImiNectDevice::getDeviceHandle() { return mImiDevice; }

bool ImiNectDevice::openDevice() {
  int ret = imiInitialize();
  if (0 != ret) {
    RCLCPP_WARN(rclcpp::get_logger("ImiNectDevice"),
                "ImiNectDevice Init Failed! ret = %d", ret);
    closeDevice();
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ImiNectDevice"),
              "ImiNectDevice Init Success");

  uint32_t deviceCount = 0;
  imiGetDeviceList(&mDeviceAttr, &deviceCount);
  if ((deviceCount <= 0) || (NULL == mDeviceAttr)) {
    RCLCPP_WARN(rclcpp::get_logger("ImiNectDevice"),
                "Get No Connected ImiNectDevice!");
    closeDevice();
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ImiNectDevice"),
              "Get %d Connected ImiNectDevice", deviceCount);

  // 3.imiOpenDevice()
  ret = imiOpenDevice(mDeviceAttr[0].uri, &mImiDevice, 0);
  if (0 != ret) {
    RCLCPP_WARN(rclcpp::get_logger("ImiNectDevice"),
                "Open ImiNectDevice Failed! ret = %d\n", ret);
    closeDevice();
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ImiNectDevice"), "ImiNectDevice Opened");
  return true;
}