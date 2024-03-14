#ifndef UTILS_HPP
#define UTILS_HPP

#include "ImiNect.h"
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
class Utils {
public:
  static std::string FrameTypeToString(ImiFrameType type) {
    std::string ret;
    switch (type) {
    case 0x00:
      ret = "Depth";
      break;
    case 0x01:
      ret = "DepthSkeleton";
      break;
    case 0x02:
      ret = "UserIdxSkeleton";
      break;
    case 0x03:
      ret = "Skeleton";
      break;
    case 0x04:
      ret = "Color";
      break;
    case 0x05:
      ret = "DepthIR";
      break;
    case 0x06:
      ret = "IR";
      break;
    case 0x09:
      ret = "IRFlood";
      break;
    default:
      break;
    }

    return ret;
  }
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(uint16_t depth) {
    return depth * 0.001f;
  } // originally mm
  static inline uint16_t fromMeters(float depth) {
    return (depth * 1000.0f) + 0.5f;
  }

  static void
  convertDepthToPoints(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                       sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg,
                       bool use_quiet_nan = false, double range_max = 0.0,
                       const float fx = 0.0, const float fy = 0.0,
                       const float cx = 0.0, const float cy = 0.0) {
    float center_x = cx;
    float center_y = cy;

    // Combine unit conversion (if necessary) with scaling by focal length for
    // computing (X,Y)
    double unit_scaling = toMeters(1);
    float constant_x = unit_scaling / fx;
    float constant_y = unit_scaling / fy;
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    const uint16_t *depth_row =
        reinterpret_cast<const uint16_t *>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(uint16_t);
    for (int v = 0; v < static_cast<int>(cloud_msg->height);
         ++v, depth_row += row_step) {
      for (int u = 0; u < static_cast<int>(cloud_msg->width);
           ++u, ++iter_x, ++iter_y, ++iter_z) {
        uint16_t depth = depth_row[u];

        // Missing points denoted by NaNs
        if (!valid(depth)) {
          if (range_max != 0.0 && !use_quiet_nan) {
            depth = fromMeters(range_max);
          } else {
            *iter_x = *iter_y = *iter_z = bad_point;
            continue;
          }
        } else if (range_max != 0.0) {
          uint16_t depth_max = fromMeters(range_max);
          if (depth > depth_max) {
            if (use_quiet_nan) {
              *iter_x = *iter_y = *iter_z = bad_point;
              continue;
            } else {
              depth = depth_max;
            }
          }
        }

        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = toMeters(depth);

        // and RGB
      }
    }
  }
};

#endif // UTILS_HPP