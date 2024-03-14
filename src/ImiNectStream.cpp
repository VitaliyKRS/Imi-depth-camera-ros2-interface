#include "imidepthcamera/ImiNectStream.hpp"
#include "imidepthcamera/Utils.hpp"
#include <rclcpp/rclcpp.hpp>

ImiNectStream::ImiNectStream(IImiNectDeviceProvider &deviceProvider,
                             ImiFrameType frameType)
    : mDeviceProvider(deviceProvider), mFrameType(frameType) {}

ImiNectStream::~ImiNectStream() { closeStream(); }

ImiImageFrame *ImiNectStream::getFrame() {

  mCurrentFrame = nullptr;
  if (mStreamHandle) {
    int ret = imiReadNextFrame(mStreamHandle, &mCurrentFrame, 100);

    if (ret != 0) {
      RCLCPP_WARN(
          rclcpp::get_logger(Utils::FrameTypeToString(mFrameType) + "Stream"),
          "Read Frame Failed! ret =  %s", imiGetErrorString(ret));
    }
  } else {
    RCLCPP_WARN(
        rclcpp::get_logger(Utils::FrameTypeToString(mFrameType) + "Stream"),
        "Stream handle is not opened!");
  }

  return mCurrentFrame;
}

void ImiNectStream::releaseFrame() { imiReleaseFrame(&mCurrentFrame); }

bool ImiNectStream::openStream() {
  auto deviceHandle = mDeviceProvider.getDeviceHandle();
  if (!deviceHandle) {
    RCLCPP_WARN(
        rclcpp::get_logger(Utils::FrameTypeToString(mFrameType) + "NectStream"),
        "Device Handle is not opened!");
    return false;
  }
  mFrameMode = imiGetCurrentFrameMode(deviceHandle, mFrameType);
  if (!mFrameMode) {
    RCLCPP_WARN(
        rclcpp::get_logger(Utils::FrameTypeToString(mFrameType) + "NectStream"),
        "Get current frame mode failed!");

    return false;
  }
  RCLCPP_INFO(
      rclcpp::get_logger(Utils::FrameTypeToString(mFrameType) + "NectStream"),
      "Get current frame mode:%d-%d-%d-%d-%d\n", mFrameMode->bitsPerPixel,
      mFrameMode->framerate, mFrameMode->pixelFormat, mFrameMode->resolutionX,
      mFrameMode->resolutionY);
  imiSetFrameMode(deviceHandle, mFrameType, (ImiFrameMode *)mFrameMode);

  int ret = imiOpenStream(deviceHandle, mFrameType, NULL, NULL, &mStreamHandle);
  if (0 != ret) {
    RCLCPP_WARN(
        rclcpp::get_logger(Utils::FrameTypeToString(mFrameType) + "NectStream"),
        "Open Depth Stream Failed! ret = %s", imiGetErrorString(ret));
    return false;
  }

  return true;
}

void ImiNectStream::closeStream() {
  if (mStreamHandle)
    imiCloseStream(mStreamHandle);
}

// void Stream::newFrameCallback(ImiStreamHandle stream, void *pUserData) {
//   RCLCPP_INFO(rclcpp::get_logger("Stream"), "New Frame arrived");
// }
