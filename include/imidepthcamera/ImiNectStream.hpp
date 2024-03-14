#ifndef IMINECTSTREAM_HPP
#define IMINECTSTREAM_HPP

#include "IImiNectDeviceProvider.hpp"
#include "IImiNectFrameProvider.hpp"
#include "IImiStream.hpp"

class ImiNectStream : public IImiStream, public IImiNectFrameProvider {
public:
  ImiNectStream(IImiNectDeviceProvider &deviceProvider, ImiFrameType frameType);
  ~ImiNectStream();

  ImiImageFrame *getFrame() override;
  void releaseFrame() override;
  bool openStream() override;
  void closeStream() override;

private:
  IImiNectDeviceProvider &mDeviceProvider;
  ImiFrameType mFrameType;
  const ImiFrameMode *mFrameMode;
  ImiStreamHandle mStreamHandle;
  ImiImageFrame *mCurrentFrame;
};

#endif // IMINECTSTREAM_HPP