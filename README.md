Ros2 interface to work with A100S+mini depth camera (Huajie Amy’s somatosensory equipment)

Preconditions:
Install Ros2 packages: https://docs.ros.org/en/humble/Installation.html
Install ImiSDK: https://drive.google.com/file/d/1bdKMtkUCiWu_xV1y85VFkQCKSyhDc8Hn/view?usp=drive_link

Usage:
Launch imidepthcamera node:
```
> ros2 launch imidepthcamera imidepthcamera.launch.py
```

Result:
Camera data over ros2 topic messages
```
> ros2 topic list
/imi_camera/color_camera_info
/imi_camera/depth_camera_info
/imi_camera/image_depth
/imi_camera/image_raw
/imi_camera/point_cloud
```

Launch parameters can be changed in config/imidepthcamera.yaml
```
imidepthcamera_node:
  ros__parameters:
    frame_id: "camera"
    publish_rate: 10.0
    publish_point_could: true
    publish_image_raw: true
    publish_image_depth: true
    publish_depth_camera_info: true
    publish_color_camera_info: true
```
