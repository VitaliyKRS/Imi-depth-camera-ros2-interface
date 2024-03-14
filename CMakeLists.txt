cmake_minimum_required(VERSION 3.8)
project(imidepthcamera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
find_package(ImiCamera REQUIRED)
find_package(ImiNect REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${ImiCamera_INCLUDE_DIRS}
  ${ImiNect_INCLUDE_DIRS}
  )

set(SOURCES   
  src/main.cpp
  src/ImiDepthCameraNode.cpp
  src/ImiCameraDevice.cpp
  src/ImiCameraStream.cpp
  src/ImiNectDevice.cpp
  src/ImiNectStream.cpp
  )



add_executable(
  ${PROJECT_NAME} ${SOURCES}
)

target_link_libraries(${PROJECT_NAME} ${ImiCamera_LIBRARY} ${ImiNect_LIBRARY} camera_info_manager::camera_info_manager)

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs std_msgs geometry_msgs)


install(TARGETS  ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
