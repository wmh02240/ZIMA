/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROS_GAZEBO_NODE_H_
#define ZIMA_ROS_GAZEBO_NODE_H_

#include <gazebo_msgs/ContactsState.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>

#include <list>

#include "zima/robot_data/nav_data.h"
#include "zima_ros/zima_node.h"

namespace zima_ros {

using namespace zima;

class ZimaGazeboNode : public ZimaNode {
 public:
  ZimaGazeboNode() = delete;
  explicit ZimaGazeboNode(const bool &use_simple_slam);
  ~ZimaGazeboNode();

  void GazeboPublishChassisControl();

  void GazeboGroundTruthOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
  void GazeboRobotScanCb(const sensor_msgs::LaserScan::ConstPtr &msg);
  void GazeboClockCb(const rosgraph_msgs::Clock::ConstPtr &msg);
  void GazeboGeneralBumperCb(const gazebo_msgs::ContactsState::ConstPtr &msg);
  void GazeboLeftWallSensorCb(const sensor_msgs::LaserScan::ConstPtr &msg);
  void GazeboRightWallSensorCb(const sensor_msgs::LaserScan::ConstPtr &msg);

  bool ProcessData(uint32_t &map_seq,
                   std::shared_ptr<Timer> &process_map_timer) override;

  void Run(const ZimaThreadWrapper::ThreadParam &param);

 private:
  // Gazebo
  ros::Publisher gazebo_robot_ctl_publisher_;
  ros::Subscriber gazebo_robot_odom_subscriber_;
  ros::Subscriber gazebo_robot_scan_subscriber_;
  ros::Subscriber gazebo_clock_subscriber_;
  ros::Subscriber gazebo_chassis_bumper_subscriber_;
  ros::Subscriber gazebo_lidar_bumper_subscriber_;
  ros::Subscriber gazebo_left_wall_sensor_subscriber_;
  ros::Subscriber gazebo_right_wall_sensor_subscriber_;

  nav_msgs::Odometry::Ptr latest_ground_truth_odom_;
  ReadWriteLock::SPtr receive_msg_list_lock_;
  std::list<nav_msgs::Odometry::ConstPtr> receive_ground_truth_odom_msg_list_;
  std::list<sensor_msgs::LaserScan::ConstPtr> receive_scan_msg_list_;
  std::list<gazebo_msgs::ContactsState::ConstPtr> receive_bumper_msg_list_;

  sensor_msgs::LaserScan::ConstPtr last_scan_msg_;
  bool enable_lidar_noise_;
  float lidar_noise_deviation_;

  double system_time_mark_;
  double sim_time_mark_;
};

}  // namespace zima_ros

#endif  // ZIMA_ROS_GAZEBO_NODE_H_
