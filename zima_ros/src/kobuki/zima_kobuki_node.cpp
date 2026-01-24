/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "kobuki/zima_kobuki_node.h"

#include "zima/common/lock.h"
#include "zima/common/util.h"
#include "zima_ros/util.h"

namespace zima_ros {

using namespace zima;

ZimaKobukiNode::ZimaKobukiNode(const std::string& port,
                               const bool& use_simple_slam)
    : ZimaNode(use_simple_slam) {
  kobuki_chassis_ = std::make_shared<KobukiChassis>(port);
  chassis_ = kobuki_chassis_;
  kobuki_chassis_->Initialize();
  InitForRos();

  ZINFO << "publish lidar tf";
  auto& tf_manager = *TransformManager::Instance();
  static_lidar_tf_broadcaster_.sendTransform(
      zima_ros::GeometryTransformToStampedTransform(
          ros::Time::now(), tf_manager.kRobotFrame_, tf_manager.kLidarFrame_,
          zima_ros::TFToGeometryMsgTransform(zima_ros::MapPointToTFTransform(
              kobuki_chassis_->GetLidar(kobuki_chassis_->kLidar_)->GetTf()))));

  robot_scan_subscriber_ =
      node.subscribe("/scan", 1, &ZimaKobukiNode::RobotScanCb, this);

  sim_time_received_.store(false);

  WriteLocker lock(odom_msg_access_);
  odom_msg_template_.reset(new nav_msgs::Odometry());
  receive_msg_list_lock_.reset(new ReadWriteLock());
}

ZimaKobukiNode::~ZimaKobukiNode(){};

void ZimaKobukiNode::RobotScanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  {
    WriteLocker scan_msg_lock(publish_scan_msg_access_);
    publish_scan_msg_ = msg;
  }

  WriteLocker lock(receive_msg_list_lock_);
  receive_scan_msg_list_.emplace_back(msg);
}

bool ZimaKobukiNode::ProcessData(
    uint32_t& map_seq, std::shared_ptr<Timer>& process_map_timer) {
  bool ret = false;
  // Process scan msgs.
  {
    WriteLocker lock(receive_msg_list_lock_);
    if (!receive_scan_msg_list_.empty()) {
      auto msg = receive_scan_msg_list_.front();
      receive_scan_msg_list_.pop_front();
      lock.Unlock();
      auto new_scan = LaserScanToPointCloud(msg);
      if (new_scan != nullptr) {
        chassis_->GetLidar(chassis_->kLidar_)
            ->UpdatePointCloudInLidarFrame(new_scan);
        slam_wrapper_->PushPointCloud(
            chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInLidarFrame());
      }

      ret = true;
    }
  }

  ret |= ZimaNode::ProcessData(map_seq, process_map_timer);
  return ret;
}

bool ZimaKobukiNode::ChassisStartThread() {
  Chassis::MergedOdomDataCb data_cb =
      [&](const MergedOdomData::SPtr& data) -> void {
    slam_wrapper_->PushMergedOdomData(data);
    // ZINFO << "Push " << data->DebugString();

    WriteLocker lock(receive_msg_list_lock_);
    odom_data_list_.emplace_back(data);
    while (!odom_data_list_.empty()) {
      if (data->GetTimeStamp() - odom_data_list_.front()->GetTimeStamp() >
          0.5) {
        odom_data_list_.pop_front();
      } else {
        break;
      }
    }
  };
  kobuki_chassis_->StartThread(data_cb);
  return true;
}

bool ZimaKobukiNode::ChassisShutdownThread() {
  return kobuki_chassis_->ShutdownThread();
}

}  // namespace zima_ros
