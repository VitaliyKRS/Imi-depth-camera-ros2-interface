import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    print(f"[DEBUG] hi")
    imidepthcamera_package = get_package_share_directory("imidepthcamera")
    calib_file = os.path.join(imidepthcamera_package, "config", "ost.yaml")
    parameter_file = os.path.join(imidepthcamera_package, "config", "imidepthcamera.yaml")
    print(f"[DEBUG] {calib_file}")
    print(f"[DEBUG] {parameter_file}")

    imidepthcamera_cmd = Node(
        package="imidepthcamera",
        executable="imidepthcamera",
        name="imidepthcamera_node",
        output="screen",
        parameters=[parameter_file, {'calib_file': calib_file}],
    )

    ld = LaunchDescription()
    ld.add_action(imidepthcamera_cmd)

    return ld
# EOF