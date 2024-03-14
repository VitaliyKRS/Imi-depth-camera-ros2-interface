
#ifndef IIMISTREAM_HPP
#define IIMISTREAM_HPP
class IImiStream {
public:
  virtual bool openStream() = 0;
  virtual void closeStream() = 0;
};

#endif // IIMISTREAM_HPP