cmake_minimum_required(VERSION 3.5)
project(nanosaur_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(/usr/include/gstreamer-1.0 /usr/include/glib-2.0 /usr/include/libxml2 /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/ /usr/lib/aarch64-linux-gnu/glib-2.0/include/ /usr/lib/x86_64-linux-gnu/glib-2.0/include/)
## System dependencies are found with CMake's conventions
find_package(jetson-utils REQUIRED)
find_package(CUDA REQUIRED)
# message("-- CUDA version: ${CUDA_VERSION}")

link_directories(${CUDA_LIBRARY_DIRS})
include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
)
add_executable(nanosaur_camera src/camera_publisher.cpp src/image_converter.cpp)
ament_target_dependencies(nanosaur_camera rclcpp std_msgs sensor_msgs vision_msgs)
target_link_libraries(nanosaur_camera jetson-utils)

install(TARGETS
  nanosaur_camera
  DESTINATION lib/${PROJECT_NAME})

ament_package()
