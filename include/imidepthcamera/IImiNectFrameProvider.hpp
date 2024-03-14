
#ifndef IIMICNECTFRAMEPROVIDER_HPP
#define IIMICNECTFRAMEPROVIDER_HPP
#include "ImiNect.h"

class IImiNectFrameProvider {
public:
  virtual ImiImageFrame *getFrame() = 0;
  virtual void releaseFrame() = 0;
};

#endif // IIMICNECTFRAMEPROVIDER_HPP