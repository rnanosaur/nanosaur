/**
 * Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <jetson-utils/gstCamera.h>

#include "nanosaur_camera/image_converter.h"


using namespace std::chrono_literals;


class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher"), frameId("base_link")
  {
    /* create image converter */
    camera_cvt = new imageConverter();
    // Load frame_id name
    this->declare_parameter<std::string>("frame_id", frameId);
    this->get_parameter("frame_id", frameId);
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frameId.c_str());
    // Initialize publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    // Initialize camera
    std::string camera_device = "0";	// MIPI CSI camera by default
    this->declare_parameter<std::string>("camera.device", camera_device);
    this->get_parameter("camera.device", camera_device);
    RCLCPP_INFO(this->get_logger(), "opening camera device %s", camera_device.c_str());
    // Load configuration
    double frameRate = 120.0;
    this->declare_parameter<double>("camera.frameRate", frameRate);
    this->get_parameter("camera.frameRate", frameRate);
    int camera_width = gstCamera::DefaultWidth;
    this->declare_parameter<int>("camera.width", camera_width);
    this->get_parameter("camera.width", camera_width);
    int camera_height = gstCamera::DefaultHeight;
    this->declare_parameter<int>("camera.height", camera_height);
    this->get_parameter("camera.height", camera_height);

    RCLCPP_INFO(this->get_logger(), "width %d height %d - Framerate %f", camera_width, camera_height, frameRate);

    videoOptions opt;
    opt.resource = camera_device.c_str();
    opt.width = camera_width;
    opt.height = camera_height;
    opt.frameRate = (float)frameRate;
    opt.ioType = videoOptions::INPUT;

    /* open camera device */
	  camera = gstCamera::Create(opt);

    if( !camera )
    {
      RCLCPP_ERROR(this->get_logger(), "failed to open camera device %s", camera_device.c_str());
    }
  }

  bool acquire()
  {
    float4* imgRGBA = NULL;
    // get the latest frame
    if( !camera->CaptureRGBA((float**)&imgRGBA, 1000) )
    {
      RCLCPP_ERROR(this->get_logger(), "failed to capture camera frame");
      return false;
    }
    // assure correct image size
    if( !camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), IMAGE_RGBA32F) )
    {
      RCLCPP_ERROR(this->get_logger(), "failed to resize camera image converter");
      return false;
    }
    // populate the message
    sensor_msgs::msg::Image msg;

    if( !camera_cvt->Convert(msg, imageConverter::ROSOutputFormat, imgRGBA) )
    {
      RCLCPP_ERROR(this->get_logger(), "failed to convert camera frame to sensor_msgs::Image");
      return false;
    }
    // Add header,timestamp and frame_id
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frameId;
    // Publish camera frame message
    publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published camera frame");
    return true;
  }

  ~CameraPublisher()
  {
    RCLCPP_DEBUG(this->get_logger(), "Close camera_publisher");
    camera->Close();
    camera_cvt->Free();
  }

private:
  gstCamera* camera;
  imageConverter* camera_cvt;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::string frameId;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Initialize Camera publisher node
  CameraPublisher camera;
  // Reference rclppp
  // https://github.com/ros2/ros_core_documentation/blob/master/source/rclcpp_cpp_client_library_overview.rst
  while (rclcpp::ok()) {
    // If acquire got an error close the stream and close the node
    if ( ! camera.acquire())
      return 1;
  }
  rclcpp::shutdown();
  return 0;
}