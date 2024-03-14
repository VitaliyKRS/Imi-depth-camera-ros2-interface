
#ifndef IIMICAMERAFRAMEPROVIDER_HPP
#define IIMICAMERAFRAMEPROVIDER_HPP
#include "ImiCamera.h"

class IImiCameraFrameProvider {
public:
  virtual ImiCameraFrame *getFrame() = 0;
  virtual void releaseFrame() = 0;
};

#endif // IIMICAMERAFRAMEPROVIDER_HPP