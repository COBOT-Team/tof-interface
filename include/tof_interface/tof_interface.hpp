#ifndef TOF_INTERFACE__ZACH_CHAMPION__HPP
#define TOF_INTERFACE__ZACH_CHAMPION__HPP

#include <stdio.h>

#include <iostream>

#include "ArducamTOFCamera.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class TOFInterface : public rclcpp::Node
{
public:
  /**
   * Construct a new TOFInterface object.
   *
   * @param[in] options The options for the node.
   */
  explicit TOFInterface(const rclcpp::NodeOptions& options);

  /**
   * Destroy the TOFInterface object. This will stop and close the TOF camera. If the camera was not
   * started, this will do nothing, so it is safe to call without checks.
   */
  ~TOFInterface();

private:
  /**
   * Convert an OpenCV image to a ROS2 message. This code is adapted from klintan's ros2_usb_camera
   * and removes cv_bridge as a dependancy.
   *
   * @param frame The OpenCV image to convert.
   * @param encoding The encoding of the image. Default is "32FC1".
   * @return std::shared_ptr<sensor_msgs::msg::Image> The ROS2 message.
   */
  std::shared_ptr<sensor_msgs::msg::Image> mat_to_msg(cv::Mat& frame,
                                                      std::string encoding = "32FC1");

  /**
   * Callback function for the image timer. This will poll the TOF camera for a new frame, waiting
   * at most 1/fps_ seconds for the frame to be ready. If the frame is not ready, the callback will
   * return without publishing the frame.
   */
  void image_callback();

  double fps_;
  std::string frame_id_;

  int period_;

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

#endif