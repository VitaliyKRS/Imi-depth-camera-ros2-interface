
#ifndef IIMINECTDEVICEPROVIDER_HPP
#define IIMINECTDEVICEPROVIDER_HPP
#include "ImiNect.h"

class IImiNectDeviceProvider {
public:
  virtual ImiDeviceHandle getDeviceHandle() = 0;
};

#endif // IIMINECTDEVICEPROVIDER_HPP