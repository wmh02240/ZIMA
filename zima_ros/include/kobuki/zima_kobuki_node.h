/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROS_KOBUKI_NODE_H_
#define ZIMA_ROS_KOBUKI_NODE_H_

#include <list>

#include "zima/hal/chassis/kobuki/chassis.h"
#include "zima_ros/zima_node.h"

namespace zima_ros {

using namespace zima;

class ZimaKobukiNode : public ZimaNode {
 public:
  ZimaKobukiNode() = delete;
  ZimaKobukiNode(const std::string &port, const bool &use_simple_slam);
  ~ZimaKobukiNode();

  void MainThread(const ZimaThreadWrapper::ThreadParam &param);

  void RobotScanCb(const sensor_msgs::LaserScan::ConstPtr &msg);

  bool ProcessData(uint32_t &map_seq,
                   std::shared_ptr<Timer> &process_map_timer) override;

  bool ChassisStartThread();
  bool ChassisShutdownThread();

 private:
  std::shared_ptr<KobukiChassis> kobuki_chassis_;
  ros::Subscriber robot_scan_subscriber_;

  ReadWriteLock::SPtr receive_msg_list_lock_;
  std::list<sensor_msgs::LaserScan::ConstPtr> receive_scan_msg_list_;

  std::deque<MergedOdomData::SPtr> odom_data_list_;
};

}  // namespace zima_ros

#endif  // ZIMA_ROS_KOBUKI_NODE_H_
