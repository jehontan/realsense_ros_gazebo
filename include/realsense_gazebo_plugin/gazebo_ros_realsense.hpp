#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <string>

namespace gazebo {
/// \brief A plugin that simulates Real Sense camera streams.
class GazeboRosRealsense : public RealSensePlugin {
  /// \brief Constructor.
public:
  GazeboRosRealsense();

  /// \brief Destructor.
public:
  ~GazeboRosRealsense();

  // Documentation Inherited.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Callback that publishes a received Depth Camera Frame as an
  /// ImageStamped message.
public:
  virtual void OnNewDepthFrame();

  /// \brief Helper function to fill the pointcloud information
  bool FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg, uint32_t rows_arg,
                            uint32_t cols_arg, uint32_t step_arg, void *data_arg);

  /// \brief Callback that publishes a received Camera Frame as an
  /// ImageStamped message.
public:
  virtual void OnNewFrame(const rendering::CameraPtr cam,
                          const transport::PublisherPtr pub);

protected:
  std::shared_ptr<camera_info_manager::CameraInfoManager>
      camera_info_manager_;

  /// \brief A pointer to the ROS node.
  ///  A node will be instantiated if it does not exist.
protected:
  std::shared_ptr<rclcpp::Node> rosnode_;

private:
  std::shared_ptr<image_transport::ImageTransport> itnode_;
  std::shared_ptr<rclcpp::Publisher> pointcloud_pub_;

protected:
  image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

  /// \brief ROS image messages
protected:
  sensor_msgs::Image image_msg_, depth_msg_;
  sensor_msgs::PointCloud2 pointcloud_msg_;
};
}
#endif /* _GAZEBO_ROS_REALSENSE_PLUGIN_ */