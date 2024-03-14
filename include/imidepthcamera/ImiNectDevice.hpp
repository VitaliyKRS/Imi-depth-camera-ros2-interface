#ifndef IMINECTDEVICE_HPP
#define IMINECTDEVICE_HPP

#include "IImiDevice.hpp"
#include "IImiNectDeviceProvider.hpp"

class ImiNectDevice : public IImiDevice, public IImiNectDeviceProvider {
public:
  ImiNectDevice();
  ~ImiNectDevice();
  bool openDevice() override;
  void closeDevice() override;
  ImiCameraIntrinsic getCameraIntrinsic();
  ImiDeviceHandle getDeviceHandle() override;

private:
  ImiDeviceAttribute *mDeviceAttr;
  ImiDeviceHandle mImiDevice;
};

#endif // IMINECTDEVICE_HPP