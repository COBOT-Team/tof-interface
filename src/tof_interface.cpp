/**
 * @file tof_interface.cpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2024-02-07
 *
 * @brief This file contains the code for interfacing with the Arducam TOF camera and publishing
 * depth and IR images to ROS2 topics.
 *
 * This code is based off of klintan's ros2_usb_camera
 * (https://github.com/klintan/ros2_usb_camera/tree/foxy-devel).
 */

#include "tof_interface/tof_interface.hpp"

using namespace std::chrono_literals;
using namespace camera_info_manager;
using namespace Arducam;

TOFInterface::TOFInterface(const rclcpp::NodeOptions& node_options)
  : Node("tof_interface", node_options)
{
  // Declare parameters.
  fps_ = this->declare_parameter<double>("fps", 30.0);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "tof_camera");
  auto calibration_file = this->declare_parameter<std::string>("calibration_file",
                                                               "package://tof_interface/"
                                                               "camera_calibration/ost.yaml");

  period_ = static_cast<int>(1000.0 / fps_);

  // Create the image transport publishers.
  depth_cinfo_pub_ = image_transport::create_camera_publisher(this, "tof/depth/image");
  ir_cinfo_pub_ = image_transport::create_camera_publisher(this, "tof/ir/image");

  // Create the camera info manager.
  cinfo_manager_ = std::make_shared<CameraInfoManager>(this);
  cinfo_manager_->loadCameraInfo(calibration_file);

  // Initialize the TOF camera.
  if (tof_camera_.open(Connection::CSI)) {
    RCLCPP_ERROR(get_logger(), "Failed to open the TOF camera");
    return;
  }
  if (tof_camera_.start()) {
    RCLCPP_ERROR(get_logger(), "Failed to start the TOF camera");
    return;
  }
  tof_camera_.setControl(CameraCtrl::RANGE, 2);  // Use 2m range; 4m doesn't work as well.
  tof_info_ = tof_camera_.getCameraInfo();
  RCLCPP_INFO(get_logger(), "TOF camera initialized");

  // Start the timer.
  last_frame_ = std::chrono::steady_clock::now();
  timer_ = this->create_wall_timer(1ms, std::bind(&TOFInterface::image_callback, this));
}

TOFInterface::~TOFInterface()
{
  tof_camera_.stop();
  tof_camera_.close();
  RCLCPP_INFO(get_logger(), "TOF camera stopped and closed");
}

std::shared_ptr<sensor_msgs::msg::Image> TOFInterface::mat_to_msg(cv::Mat& frame,
                                                                  std::string encoding)
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image image;

  image.header = header;
  image.height = frame.rows;
  image.width = frame.cols;
  image.encoding = encoding;
  image.is_bigendian = false;  // TODO: Determine this programmatically.
  image.step = frame.cols * frame.elemSize();
  size_t size = image.step * frame.rows;
  image.data.resize(size);

  if (frame.isContinuous())
    memcpy(reinterpret_cast<uint8_t*>(&image.data[0]), frame.data, size);
  else {
    uint8_t* image_data = reinterpret_cast<uint8_t*>(&image.data[0]);
    uint8_t* frame_data = frame.data;
    for (int i = 0; i < frame.rows; ++i) {
      memcpy(image_data, frame_data, image.step);
      image_data += image.step;
      frame_data += frame.step;
    }
  }

  return std::make_shared<sensor_msgs::msg::Image>(image);
}

void TOFInterface::image_callback()
{
  // If the time since the last frame is less than the period, return without doing anything.
  auto now = std::chrono::steady_clock::now();
  auto next_frame = last_frame_ + std::chrono::milliseconds(period_);
  auto delay = next_frame - now;
  if (delay > 0ms) return;

  // Grab the frame from the TOF camera. If the frame is not ready, return without publishing the
  // frame.
  frame_buffer_ = tof_camera_.requestFrame(0);
  if (frame_buffer_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to get frame from TOF camera");
    tof_camera_.releaseFrame(frame_buffer_);
    return;
  }
  last_frame_ = now;

  // Convert our frames to OpenCV images.
  float* depth_ptr = (float*)frame_buffer_->getData(FrameType::DEPTH_FRAME);
  float* ir_ptr = (float*)frame_buffer_->getData(FrameType::AMPLITUDE_FRAME);
  depth_frame_ = cv::Mat(tof_info_.height, tof_info_.width, CV_32FC1, depth_ptr);
  ir_frame_ = cv::Mat(tof_info_.height, tof_info_.width, CV_32FC1, ir_ptr);

  // Convert our OpenCV images to ROS2 messages.
  depth_msg_ = mat_to_msg(depth_frame_, "32FC1");
  ir_msg_ = mat_to_msg(ir_frame_, "32FC1");

  // Create a new CameraInfo message. I'm not entirely sure if this is necessary; I think it can
  // be done once and then reused. However, the USB camera driver that I'm basing this off of
  // does it like this, so I will too.
  sensor_msgs::msg::CameraInfo::SharedPtr cinfo_msg_(
      new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

  // Set the frame ID and timestamp for the messages.
  rclcpp::Time timestamp = this->get_clock()->now();
  depth_msg_->header.frame_id = frame_id_;
  depth_msg_->header.stamp = timestamp;
  ir_msg_->header.frame_id = frame_id_;
  ir_msg_->header.stamp = timestamp;
  cinfo_msg_->header.frame_id = frame_id_;
  cinfo_msg_->header.stamp = timestamp;

  depth_cinfo_pub_.publish(depth_msg_, cinfo_msg_);
  ir_cinfo_pub_.publish(ir_msg_, cinfo_msg_);

  tof_camera_.releaseFrame(frame_buffer_);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<TOFInterface>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
