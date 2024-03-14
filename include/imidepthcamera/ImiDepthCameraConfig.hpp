#include <string>

struct ImiDepthCameraConfig {
  std::string mFrameId;
  double mPublishRate;
  bool mPublishPointCloud;
  bool mPublishImageRaw;
  bool mPublishImageDepth;
  bool mPublishDepthCameraInfo;
  bool mPublishColorCameraInfo;
  std::string mCalibrationFile;
};