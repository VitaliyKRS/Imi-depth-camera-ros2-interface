#ifndef IIMICAMERADEVICEPROVIDER_HPP
#define IIMICAMERADEVICEPROVIDER_HPP

#include "ImiCamera.h"

class IImiCameraDeviceProvider {
public:
  virtual ImiCameraHandle getDeviceHandle() = 0;
};

#endif // IIMICAMERADEVICEPROVIDER_HPP