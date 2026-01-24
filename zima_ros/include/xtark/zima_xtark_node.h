/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROS_XTARK_NODE_H_
#define ZIMA_ROS_XTARK_NODE_H_

#include "zima/hal/chassis/xtark/chassis.h"
#include "zima_ros/zima_node.h"

namespace zima_ros {

using namespace zima;

class ZimaXTarkNode : public ZimaNode {
 public:
  ZimaXTarkNode() = delete;
  explicit ZimaXTarkNode(const bool& use_simple_slam);
  ~ZimaXTarkNode();

  void MainThread();

  void ReadThread();
  void WriteThread();
  void ProcessDataThread();

  void RobotScanCb(const sensor_msgs::LaserScan::ConstPtr& msg);

 private:
  std::shared_ptr<XTarkChassis> xtark_chassis_;
  std::shared_ptr<XTarkChassisSerial> serial_;
  // ros::Subscriber gazebo_robot_odom_subscriber_;
  // ros::Subscriber gazebo_robot_sensor_subscriber_;
  ros::Subscriber robot_scan_subscriber_;
  // ros::Subscriber gazebo_world_stats_subscriber_;
};

}  // namespace zima_ros

#endif  // ZIMA_ROS_XTARK_NODE_H_
