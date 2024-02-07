/**
 * @file tof_interface.cpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2024-02-07
 *
 * @brief This file contains the code for interfacing with the Arducam TOF camera and publishing
 * depth and IR images to ROS2 topics.
 */

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "ArducamTOFCamera.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"

using namespace std::chrono_literals;
using namespace Arducam;

int main(int argc, char** argv)
{
  // Setup the ROS2 node and image transport
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tof_interface", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher depth_pub = it.advertise("tof/depth", 1);
  image_transport::Publisher ir_pub = it.advertise("tof/ir", 1);

  // Initialize the TOF camera
  ArducamTOFCamera tof_camera;
  if (tof_camera.open(Connection::CSI)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open the TOF camera");
    return 1;
  }
  if (tof.start()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to start the TOF camera");
    return 1;
  }
  tof.setControl(CameraCtrl::RANGE, 2);  // Use 2m range; 4m doesn't work as well
  RCLCPP_INFO(node->get_logger(), "TOF camera initialized");

  // Main loop
  cv::Mat depth_image, ir_image;
  std_msgs::msg::Header depth_header, ir_header;
  sensor_msgs::msg::Image::SharedPtr depth_msg, ir_msg;
  rclcpp::WallRate loop_rate(30);
  ArducamFrameBuffer* frame;

  while (rclcpp::ok()) {
    // Get the next frame from the TOF camera
    frame = tof_camera.requestFrame(200);
    if (frame == nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get frame from TOF camera");
      continue;
    }
    float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    float* ir_ptr = (uint8_t*)frame->getData(FrameType::AMPLITUDE_FRAME);

    // Convert the depth and IR data to OpenCV images
    depth_image = cv::Mat(tof_camera.getHeight(), tof_camera.getWidth(), CV_32FC1, depth_ptr);
    ir_image = cv::Mat(tof_camera.getHeight(), tof_camera.getWidth(), CV_32FC1, ir_ptr);

    // Convert the OpenCV images to ROS2 messages
    depth_msg = cv_bridge::CvImage(depth_header, "32FC1", depth_image).toImageMsg();
    ir_msg = cv_bridge::CvImage(ir_header, "32FC1", ir_image).toImageMsg();

    // Publish the depth and IR images
    depth_pub.publish(depth_msg);
    ir_pub.publish(ir_msg);

    // Release the frame and spin the node
    tof_camera.releaseFrame(frame);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  tof.stop();
  tof.close();
  return 0;
}
