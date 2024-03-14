#include "imidepthcamera/ImiDepthCameraNode.hpp"
#include "imidepthcamera/Utils.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;
ImiDepthCameraNode::ImiDepthCameraNode()
    : Node("ImiDepthCameraNode"), mCameraStream(mCameraDevice),
      mNectStream(mNectDevice, IMI_DEPTH_FRAME) {
  configure();

  mCameraDevice.openDevice();
  mCameraStream.openStream();
  mNectDevice.openDevice();
  mNectStream.openStream();

  createCameraInfoMsgs();

  mCameraInfoPublishers[COLOR] =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(
          "/imi_camera/color_camera_info", 10);
  mCameraInfoPublishers[DEPTH] =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(
          "/imi_camera/depth_camera_info", 10);
  mImagePublishers[COLOR] = this->create_publisher<sensor_msgs::msg::Image>(
      "/imi_camera/image_raw", 10);
  mImagePublishers[DEPTH] = this->create_publisher<sensor_msgs::msg::Image>(
      "/imi_camera/image_depth", 10);

  mPointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/imi_camera/point_cloud", 10);

  mTimers.insert(this->create_wall_timer(
      10ms, std::bind(&ImiDepthCameraNode::publishImage, this)));
  mTimers.insert(this->create_wall_timer(
      10ms, std::bind(&ImiDepthCameraNode::publishDepthImage, this)));
  mTimers.insert(this->create_wall_timer(
      30ms, std::bind(&ImiDepthCameraNode::publishCameraInfo, this)));
}
void ImiDepthCameraNode::configure() {
  declare_parameter("frame_id", "");
  declare_parameter("publish_rate", 10.0);
  declare_parameter("publish_point_could", false);
  declare_parameter("publish_image_raw", false);
  declare_parameter("publish_image_depth", false);
  declare_parameter("publish_depth_camera_info", false);
  declare_parameter("publish_color_camera_info", false);
  declare_parameter("calib_file", "");

  mCameraConfig.mFrameId = get_parameter("frame_id").as_string();
  mCameraConfig.mPublishRate = get_parameter("publish_rate").as_double();
  mCameraConfig.mPublishPointCloud =
      get_parameter("publish_point_could").as_bool();
  mCameraConfig.mPublishImageRaw = get_parameter("publish_image_raw").as_bool();
  mCameraConfig.mPublishImageDepth =
      get_parameter("publish_image_depth").as_bool();
  mCameraConfig.mPublishDepthCameraInfo =
      get_parameter("publish_depth_camera_info").as_bool();
  mCameraConfig.mPublishColorCameraInfo =
      get_parameter("publish_color_camera_info").as_bool();
  mCameraConfig.mCalibrationFile =
      "file://" + get_parameter("calib_file").as_string();
}
void ImiDepthCameraNode::createCameraInfoMsgs() {
  auto depthIntrinsic = mNectDevice.getCameraIntrinsic();
  mCameraInfoMsgs[DEPTH].header.frame_id = mCameraConfig.mFrameId;
  mCameraInfoMsgs[DEPTH].header.stamp = this->now();
  mCameraInfoMsgs[DEPTH].width = 640;
  mCameraInfoMsgs[DEPTH].height = 480;
  mCameraInfoMsgs[DEPTH].distortion_model = "plumb_bob";
  mCameraInfoMsgs[DEPTH].k = {depthIntrinsic.fxParam,
                              0.0,
                              depthIntrinsic.cxParam,
                              0.0,
                              depthIntrinsic.fyParam,
                              depthIntrinsic.cyParam,
                              0.0,
                              0.0,
                              1.0};
  mCameraInfoMsgs[DEPTH].r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  mCameraInfoMsgs[DEPTH].p = {depthIntrinsic.fxParam,
                              0.0,
                              depthIntrinsic.cxParam,
                              0.0,
                              0.0,
                              depthIntrinsic.fyParam,
                              depthIntrinsic.cyParam,
                              0.0,
                              0.0,
                              0.0,
                              1.0,
                              0.0};
  mCameraInfoMsgs[DEPTH].d = {0.0, 0.0, 0.0, 0.0, 0.0};
  mCameraInfoMsgs[DEPTH].binning_x = 0;
  mCameraInfoMsgs[DEPTH].binning_y = 0;

  mCameraInfoMsgs[DEPTH].roi.x_offset = 0;
  mCameraInfoMsgs[DEPTH].roi.y_offset = 0;
  mCameraInfoMsgs[DEPTH].roi.height = 0;
  mCameraInfoMsgs[DEPTH].roi.width = 0;
  mCameraInfoMsgs[DEPTH].roi.do_rectify = false;

  camera_info_manager::CameraInfoManager c(this);
  c.validateURL(mCameraConfig.mCalibrationFile);
  c.loadCameraInfo(mCameraConfig.mCalibrationFile);
  mCameraInfoMsgs[COLOR] = c.getCameraInfo();

  mCameraInfoMsgs[COLOR].header.frame_id = mCameraConfig.mFrameId;
  mCameraInfoMsgs[COLOR].header.stamp = this->now();
}
void ImiDepthCameraNode::publishCameraInfo() {
  if (mCameraConfig.mPublishColorCameraInfo) {
    mCameraInfoMsgs[COLOR].header.stamp = this->now();
    mCameraInfoPublishers[COLOR]->publish(mCameraInfoMsgs[COLOR]);
  }
  if (mCameraConfig.mPublishColorCameraInfo) {
    mCameraInfoMsgs[DEPTH].header.stamp = this->now();
    mCameraInfoPublishers[DEPTH]->publish(mCameraInfoMsgs[DEPTH]);
  }
}

void ImiDepthCameraNode::publishImage() {
  if (mCameraConfig.mPublishImageRaw) {
    auto frame = mCameraStream.getFrame();

    if (!frame) {
      RCLCPP_WARN(this->get_logger(), "Cannot get frame!");
      return;
    }

    auto imageMsg = sensor_msgs::msg::Image();
    imageMsg.header.frame_id = mCameraConfig.mFrameId;
    imageMsg.header.stamp = this->now();

    imageMsg.width = frame->width;
    imageMsg.height = frame->height;
    imageMsg.data.resize(frame->size);
    std::memcpy(imageMsg.data.data(), frame->pData, frame->size);
    imageMsg.encoding = "rgb8";
    imageMsg.step = frame->width * 3;
    imageMsg.is_bigendian = false;

    mImagePublishers[COLOR]->publish(imageMsg);

    mCameraStream.releaseFrame();
  }
}

void ImiDepthCameraNode::publishDepthImage() {
  auto frame = mNectStream.getFrame();

  if (!frame) {
    RCLCPP_WARN(this->get_logger(), "Cannot get frame!");
    return;
  }

  sensor_msgs::msg::Image::SharedPtr imageMsg =
      std::make_shared<sensor_msgs::msg::Image>();
  imageMsg->header.frame_id = mCameraConfig.mFrameId;
  imageMsg->header.stamp = this->now();

  imageMsg->width = frame->width;
  imageMsg->height = frame->height;
  imageMsg->data.resize(frame->size);
  std::memcpy(imageMsg->data.data(), frame->pData, frame->size);
  imageMsg->encoding = "16UC1";
  imageMsg->step = frame->width * sizeof(uint16_t);
  imageMsg->is_bigendian = false;

  if (mCameraConfig.mPublishPointCloud) {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header = imageMsg->header;
    cloud_msg->height = imageMsg->height;
    cloud_msg->width = imageMsg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;
    cloud_msg->fields.clear();
    cloud_msg->fields.reserve(2);
    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    Utils::convertDepthToPoints(
        imageMsg, cloud_msg, 1.0, false, mCameraInfoMsgs[DEPTH].k[0],
        mCameraInfoMsgs[DEPTH].k[4], mCameraInfoMsgs[DEPTH].k[2],
        mCameraInfoMsgs[DEPTH].k[5]);

    mPointCloudPublisher->publish(*cloud_msg);
  }
  if (mCameraConfig.mPublishImageDepth) {
    mImagePublishers[DEPTH]->publish(*imageMsg);
  }

  mNectStream.releaseFrame();
}