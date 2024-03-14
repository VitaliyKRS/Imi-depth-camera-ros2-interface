#include "imidepthcamera/ImiDepthCameraNode.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImiDepthCameraNode>());
  rclcpp::shutdown();
  return 0;
}