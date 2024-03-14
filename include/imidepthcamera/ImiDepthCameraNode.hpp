#ifndef IMIDEPTHCAMERANODE_HPP
#define IMIDEPTHCAMERANODE_HPP

#include "ImiCameraDevice.hpp"
#include "ImiCameraStream.hpp"
#include "ImiDepthCameraConfig.hpp"
#include "ImiNectDevice.hpp"
#include "ImiNectStream.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <unordered_set>

class ImiDepthCameraNode : public rclcpp::Node {
public:
  ImiDepthCameraNode();

private:
  enum Stream : uint8_t {
    COLOR = 0,
    DEPTH,
  };
  void configure();
  void createCameraInfoMsgs();
  void publishImage();
  void publishDepthImage();
  void publishCameraInfo();

  ImiDepthCameraConfig mCameraConfig;

  ImiCameraDevice mCameraDevice;
  ImiCameraStream mCameraStream;
  ImiNectDevice mNectDevice;
  ImiNectStream mNectStream;
  std::unordered_map<uint8_t, sensor_msgs::msg::CameraInfo> mCameraInfoMsgs;
  std::unordered_map<uint8_t,
                     rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      mCameraInfoPublishers;
  std::unordered_map<uint8_t,
                     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      mImagePublishers;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mPointCloudPublisher;

  std::unordered_set<rclcpp::TimerBase::SharedPtr> mTimers;
};

#endif // IMIDEPTHCAMERANODE_HPP