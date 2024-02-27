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

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"

using namespace std::chrono_literals;
using namespace camera_info_manager;

class TOFInterface : public rclcpp::Node
{
public:
  /**
   * Construct a new TOFInterface object.
   *
   * @param[in] options The options for the node.
   */
  explicit TOFInterface(const rclcpp::NodeOptions& options) : frame_misses_(0),
  {
    // Declare parameters.
    fps_ = this->declare_parameter<double>("fps", 30.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "tof_camera");
    auto calibration_file = this->declare_parameter<std::string>("calibration_file",
                                                                 "package://tof_interface/"
                                                                 "camera_calibration/ost.yaml");

    period_ = 1000.0 / fps_;

    // Create the image transport publishers.
    depth_cinfo_pub_ = image_transport::create_camera_publisher(this, "tof/depth/image");
    ir_cinfo_pub_ = image_transport::create_camera_publisher(this, "tof/ir/image");

    // Create the camera info manager.
    cinfo_manager_ = std::make_shared<CameraInfoManager>(this);
    cinfo_manager_->loadCameraInfo(calibration_file);

    // Initialize the TOF camera.
    if (tof_camera_.open(Connection::CSI)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to open the TOF camera");
      return;
    }
    if (tof_camera_.start()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to start the TOF camera");
      return;
    }
    tof_camera_.setControl(CameraCtrl::RANGE, 2);  // Use 2m range; 4m doesn't work as well.
    tof_info_ = tof_camera_.getCameraInfo();
    RCLCPP_INFO(node->get_logger(), "TOF camera initialized");

    // Start the timer.
    last_frame_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(1ms, std::bind(&TOFInterface::image_callback, this));
  }

  /**
   * Destroy the TOFInterface object. This will stop and close the TOF camera. If the camera was not
   * started, this will do nothing, so it is safe to call without checks.
   */
  ~TOFInterface()
  {
    tof_camera_.stop();
    tof_camera_.close();
  }

private:
  /**
   * Callback function for the image timer. This will poll the TOF camera for a new frame, waiting
   * at most 1/fps_ seconds for the frame to be ready. If the frame is not ready, the callback will
   * return without publishing the frame.
   */
  void image_callback()
  {
    // Grab the frame from the TOF camera. If the frame is not ready, return without publishing the
    // frame.
    frame_buffer_ = tof_camera.requestFrame(static_cast<int>(period_));
    if (frame_buffer_ == nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get frame from TOF camera");
      return;
    }

    // Make sure we are not publishing frames faster than the desired FPS.
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_);
    if (duration.count() < period_) return;
    last_frame_ = now;

    // Convert our frames to OpenCV images.
    float* depth_ptr = (float*)frame_buffer_->getData(FrameType::DEPTH_FRAME);
    float* ir_ptr = (float*)frame_buffer_->getData(FrameType::AMPLITUDE_FRAME);
    depth_frame_ = cv::Mat(tof_info_.height, tof_info_.width, CV_32FC1, depth_ptr);
    ir_frame_ = cv::Mat(tof_inf_o.height, tof_info_.width, CV_32FC1, ir_ptr);

    // Convert our OpenCV images to ROS2 messages.
    depth_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_frame_).toImageMsg();
    ir_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", ir_frame_).toImageMsg();

    // Create a new CameraInfo message. I'm not entirely sure if this is necessary; I think it can
    // be done once and then reused. However, the USB camera driver that I'm basing this off of
    // does it like this, so I will too.
    sensor_msgs::CameraInfo::SharedPtr cinfo_msg_(
        new sensor_msgs::CameraInfo(cinfo_manager_->getCameraInfo()));

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

  double fps_;
  std::string frame_id_;

  double period_;

  Arducam::ArducamTOFCamera tof_camera_;
  Arducam::CameraInfo tof_info_;
  Arducam::ArducamFrameBuffer* frame_buffer_;

  cv::Mat depth_frame_;
  cv::Mat ir_frame_;

  std::chrono::steady_clock::time_point last_frame_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
  image_transport::CameraPublisher depth_cinfo_pub_;
  image_transport::CameraPublisher ir_cinfo_pub_;

  std::shared_ptr<sensor_msgs::msg::Image> depth_msg_;
  std::shared_ptr<sensor_msgs::msg::Image> ir_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<TOFInterface>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
