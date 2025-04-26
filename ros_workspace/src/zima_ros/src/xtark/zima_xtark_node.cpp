/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "xtark/zima_xtark_node.h"

#include "zima/common/util.h"
#include "zima_ros/util.h"

namespace zima_ros {

using namespace zima;

ZimaXTarkNode::ZimaXTarkNode(const bool& use_simple_slam)
    : ZimaNode(use_simple_slam) {
  xtark_chassis_ = std::make_shared<XTarkChassis>();
  chassis_ = xtark_chassis_;
  xtark_chassis_->Initialize();
  InitForAutoCleaning();
  InitForRos();

  ZINFO << "publish lidar tf";
  auto& tf_manager = *TransformManager::Instance();
  static_lidar_tf_broadcaster_.sendTransform(
      zima_ros::GeometryTransformToStampedTransform(
          ros::Time::now(), tf_manager.kRobotFrame_, tf_manager.kLidarFrame_,
          zima_ros::TFToGeometryMsgTransform(zima_ros::MapPointToTFTransform(
              xtark_chassis_->GetLidar(xtark_chassis_->kLidar_)->GetTf()))));

  serial_.reset(new XTarkChassisSerial("/dev/ttyAMA0", 115200));
  robot_scan_subscriber_ =
      node.subscribe("/scan", 1, &ZimaXTarkNode::RobotScanCb, this);

  sim_time_received_.store(false);
}

ZimaXTarkNode::~ZimaXTarkNode() { serial_.reset(); };

void ZimaXTarkNode::RobotScanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  PointCloud::SPtr new_scan(
      new PointCloud(Lidar::kPointCloudInLidarFrameName_));
  WriteLocker scan_lock(new_scan->GetLock());
  new_scan->SetTimeStamp(Time::Now());
  new_scan->SetSeq(msg->header.seq);
  if (msg->ranges.size() != msg->intensities.size()) {
    ZWARN << "size " << msg->ranges.size() << " vs " << msg->intensities.size();
    return;
  }
  auto& points = new_scan->GetPointsRef();
  points.clear();
  for (auto i = 0u; i < msg->ranges.size(); i++) {
    if (msg->ranges.at(i) > 30) {
      continue;
    }
    points.emplace_back(PointCloud::Point(
        msg->ranges.at(i),
        RadiansToDegrees(msg->angle_min + msg->angle_increment * i),
        msg->intensities.at(i)));
  }
  scan_lock.Unlock();

  chassis_->GetLidar(chassis_->kLidar_)->UpdatePointCloudInLidarFrame(new_scan);
  slam_wrapper_->PushPointCloud(
      chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInLidarFrame());

  // ZINFO;
  {
    ReadLocker scan_lock(new_scan->GetLock());
    const float kTriggerDistance = chassis_->GetRadius() + 0.02;

    auto start_degree =
        chassis_->GetBumper(chassis_->kLeftBumper_)->GetCoverRangeMinDegree();
    auto end_degree =
        chassis_->GetBumper(chassis_->kLeftBumper_)->GetCoverRangeMaxDegree();
    bool triggered = false;
    new_scan->ProcessRangedPoints(
        start_degree, end_degree, [&](const PointCloud::Point& point) {
          if (fabs(point.Y()) < chassis_->GetRadius() * 1.1 &&
              point.Distance() < kTriggerDistance) {
            triggered = true;
          }
        });
    chassis_->GetBumper(chassis_->kLeftBumper_)
        ->UpdateRealtimeTriggeredState(triggered);
    if (triggered) {
      ZWARN << "left bumper.";
    }

    start_degree =
        chassis_->GetBumper(chassis_->kLeftBumper_)->GetCoverRangeMinDegree();
    end_degree =
        chassis_->GetBumper(chassis_->kLeftBumper_)->GetCoverRangeMaxDegree();
    triggered = false;
    new_scan->ProcessRangedPoints(
        start_degree, end_degree, [&](const PointCloud::Point& point) {
          if (fabs(point.Y()) < chassis_->GetRadius() * 1.1 &&
              point.Distance() < kTriggerDistance) {
            triggered = true;
          }
        });
    chassis_->GetBumper(chassis_->kRightBumper_)
        ->UpdateRealtimeTriggeredState(triggered);
    if (triggered) {
      ZWARN << "right bumper.";
    }
  }
}

}  // namespace zima_ros
