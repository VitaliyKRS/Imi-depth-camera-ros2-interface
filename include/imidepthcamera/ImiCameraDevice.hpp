#ifndef IMICAMERADEVICE_HPP
#define IMICAMERADEVICE_HPP

#include "IImiCameraDeviceProvider.hpp"
#include "IImiDevice.hpp"

class ImiCameraDevice : public IImiDevice, public IImiCameraDeviceProvider {
public:
  ImiCameraDevice();
  ~ImiCameraDevice();
  bool openDevice() override;
  void closeDevice() override;
  ImiCameraHandle getDeviceHandle() override;

private:
  ImiCameraHandle mImiDevice;
};
#endif // IMICAMERADEVICE_HPP