#include "imidepthcamera/ImiCameraStream.hpp"

#include <rclcpp/rclcpp.hpp>

ImiCameraStream::ImiCameraStream(IImiCameraDeviceProvider &deviceProvider)
    : mDeviceProvider(deviceProvider) {}

ImiCameraStream::~ImiCameraStream() { closeStream(); }

ImiCameraFrame *ImiCameraStream::getFrame() {

  mCurrentFrame = nullptr;
  auto deviceHandle = mDeviceProvider.getDeviceHandle();
  if (deviceHandle) {

    int ret = imiCamReadNextFrame(deviceHandle, &mCurrentFrame, 100);
    if (ret != 0) {
      RCLCPP_WARN(rclcpp::get_logger("ColorCameraStream"),
                  "Read Frame Failed! ret =  %d", ret);
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ColorCameraStream"),
                "Device Handle is not opened!");
  }

  return mCurrentFrame;
}

void ImiCameraStream::releaseFrame() { imiCamReleaseFrame(&mCurrentFrame); }

bool ImiCameraStream::openStream() {
  auto deviceHandle = mDeviceProvider.getDeviceHandle();
  if (!deviceHandle) {
    RCLCPP_WARN(rclcpp::get_logger("ColorCameraStream"),
                "Device Handle is not opened!");
    return false;
  }

  mFrameMode = imiCamGetCurrentFrameMode(deviceHandle);

  if (!mFrameMode) {
    RCLCPP_WARN(rclcpp::get_logger("ColorCameraStream"),
                "Get current frame mode failed!");

    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("ColorCameraStream"),
              "Get current frame mode:%d-%d-%d-%d\n", mFrameMode->fps,
              mFrameMode->pixelFormat, mFrameMode->resolutionX,
              mFrameMode->resolutionY);

  int ret = imiCamStartStream(deviceHandle, mFrameMode);

  if (0 != ret) {
    RCLCPP_WARN(rclcpp::get_logger("ColorCameraStream"),
                "Open Camera Stream Failed! ret = %d", ret);
    return false;
  }

  return true;
}

void ImiCameraStream::closeStream() {
  auto deviceHandle = mDeviceProvider.getDeviceHandle();
  if (!deviceHandle) {
    RCLCPP_WARN(rclcpp::get_logger("ColorCameraStream"),
                "Device Handle is not opened!");
    return;
  }
  imiCamStopStream(deviceHandle);
}