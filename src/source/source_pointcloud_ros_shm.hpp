/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "source/source.hpp"

#ifdef ROS_FOUND

static_assert(1 < 0, "Sorry, zero-copy is not supported in ROS1, please use ROS2 instead.");

#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "shm_msgs/msg/point_cloud2.hpp"
#include "shm_msgs/point_cloud2_iterator.hpp"
#include <sstream>
#include <stdio.h>

using Topic = shm_msgs::msg::PointCloud1m;
std::string topic_suffix = "_shm1m";

namespace robosense
{
namespace lidar
{

inline void toRosMsg(const LidarPointCloudMsg& rs_msg, const std::string& frame_id, Topic& ros_shm_msg)
{
  // int fields = 4;
#ifdef POINT_TYPE_XYZIRT
  // fields = 6;
#endif
  // ros_shm_msg.fields.clear();
  ros_shm_msg.fields_size = 0;
  // ros_shm_msg.fields.reserve(fields);

  int offset = 0;
  offset = addPointField(ros_shm_msg, "x", 1, shm_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_shm_msg, "y", 1, shm_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_shm_msg, "z", 1, shm_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_shm_msg, "intensity", 1, shm_msgs::msg::PointField::UINT8, offset);
#ifdef POINT_TYPE_XYZIRT
  offset = addPointField(ros_shm_msg, "ring", 1, shm_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_shm_msg, "timestamp", 1, shm_msgs::msg::PointField::FLOAT64, offset);
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
  printf("the fields_size is: %d \n", ros_shm_msg.fields_size);
#endif

  ros_shm_msg.point_step = offset;
  ros_shm_msg.width = rs_msg.height;
  ros_shm_msg.height = rs_msg.width;
  ros_shm_msg.row_step = ros_shm_msg.width * ros_shm_msg.point_step;
  ros_shm_msg.is_dense = rs_msg.is_dense;

  // ros_shm_msg.data.resize(ros_shm_msg.point_step * ros_shm_msg.width * ros_shm_msg.height);

  shm_msgs::PointCloud2Iterator1m<float> iter_x_(ros_shm_msg, "x");
  shm_msgs::PointCloud2Iterator1m<float> iter_y_(ros_shm_msg, "y");
  shm_msgs::PointCloud2Iterator1m<float> iter_z_(ros_shm_msg, "z");
  shm_msgs::PointCloud2Iterator1m<uint8_t> iter_intensity_(ros_shm_msg, "intensity");
#ifdef POINT_TYPE_XYZIRT
  shm_msgs::PointCloud2Iterator1m<uint16_t> iter_ring_(ros_shm_msg, "ring");
  shm_msgs::PointCloud2Iterator1m<double> iter_timestamp_(ros_shm_msg, "timestamp");
#endif

  // check the data size
  auto data_size = ros_shm_msg.point_step * ros_shm_msg.width * ros_shm_msg.height;
  if(data_size > Topic::DATA_MAX_SIZE)
  {
    std::stringstream ss;
    ss << "PCLPointCloud2 is wrongly formed: actual size which is " << data_size << " > "
       << Topic::DATA_MAX_SIZE
       << " which is the maximum size";
    throw std::runtime_error(ss.str());
  }

  // copy the data
  for (size_t i = 0; i < rs_msg.points.size(); i++)
  {
    const LidarPointCloudMsg::PointT& point = rs_msg.points[i];

    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;

    iter_x_ = iter_x_ + 1;
    iter_y_ = iter_y_ + 1;
    iter_z_ = iter_z_ + 1;
    iter_intensity_ = iter_intensity_ + 1;

#ifdef POINT_TYPE_XYZIRT
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timestamp;

    iter_ring_ = iter_ring_ + 1;
    iter_timestamp_ = iter_timestamp_ + 1;
#endif
  }

  ros_shm_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_shm_msg.header.stamp.nanosec = (uint32_t)round((rs_msg.timestamp - ros_shm_msg.header.stamp.sec) * 1e9);
  shm_msgs::set_str(ros_shm_msg.header.frame_id, frame_id);

}

class DestinationPointCloudRosShm : virtual public DestinationPointCloud
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPointCloud(const LidarPointCloudMsg& rs_msg);
  virtual ~DestinationPointCloudRosShm() = default;

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<Topic>::SharedPtr pub_;
  std::string frame_id_;

  void populateLoanedMessage(const LidarPointCloudMsg& rs_msg, rclcpp::LoanedMessage<Topic> &loanedMsg)
  {
    Topic &ros_shm_msg = loanedMsg.get();
    toRosMsg(rs_msg, frame_id_, ros_shm_msg);
    // ros_shm_msg.header.stamp = node_ptr_->get_clock()->now();;
    // set_str(ros_shm_msg.header.frame_id, "rslidar");
  }
};

inline void DestinationPointCloudRosShm::init(const YAML::Node& config)
{
  yamlRead<std::string>(config["ros"],
      "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"],
      "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "rslidar_points_destination_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));
  pub_ = node_ptr_->create_publisher<Topic>(ros_send_topic+topic_suffix, 10);
}

inline void DestinationPointCloudRosShm::sendPointCloud(const LidarPointCloudMsg& rs_msg)
{
  auto loanedMsg = pub_->borrow_loaned_message();
  populateLoanedMessage(rs_msg, loanedMsg);
  pub_->publish(std::move(loanedMsg));
}

}  // namespace lidar
}  // namespace robosense

#endif

