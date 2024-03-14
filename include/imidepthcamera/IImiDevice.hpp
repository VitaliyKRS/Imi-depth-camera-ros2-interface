#ifndef IIMIDEVICE_HPP
#define IIMIDEVICE_HPP

class IImiDevice {
public:
  virtual bool openDevice() = 0;
  virtual void closeDevice() = 0;
};

#endif // IIMIDEVICE_HPP