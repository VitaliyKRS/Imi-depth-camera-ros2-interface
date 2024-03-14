#ifndef IMICAMERASTREAM_HPP
#define IMICAMERASTREAM_HPP

#include "IImiCameraDeviceProvider.hpp"
#include "IImiCameraFrameProvider.hpp"
#include "IImiStream.hpp"

class ImiCameraStream : public IImiStream, public IImiCameraFrameProvider {
public:
  ImiCameraStream(IImiCameraDeviceProvider &deviceProvider);
  ~ImiCameraStream();

  ImiCameraFrame *getFrame() override;
  void releaseFrame() override;
  bool openStream() override;
  void closeStream() override;

private:
  IImiCameraDeviceProvider &mDeviceProvider;
  const ImiCameraFrameMode *mFrameMode;
  ImiCameraFrame *mCurrentFrame;
};

#endif // IMICAMERASTREAM_HPP