/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "gazebo/zima_gazebo_node.h"

#include <random>

#include "zima/hal/system/cpu.h"
#include "zima_ros/util.h"

namespace zima_ros {

using namespace zima;

ZimaGazeboNode::ZimaGazeboNode(const bool &use_simple_slam)
    : ZimaNode(use_simple_slam),
      enable_lidar_noise_(true),
      lidar_noise_deviation_(0.003),
      system_time_mark_(-1),
      sim_time_mark_(-1) {
  InitForChassis();
  InitForRos();

  if (IsRunningOnARM()) {
    ZINFO << "Run on ARM.";
  }

  ZINFO << "Create lidar tf: "
        << chassis_->GetLidar(chassis_->kLidar_)->GetTf().DebugString();
  auto &tf_manager = *TransformManager::Instance();
  static_lidar_tf_broadcaster_.sendTransform(
      zima_ros::GeometryTransformToStampedTransform(
          ros::Time::now(), tf_manager.kRobotFrame_, tf_manager.kLidarFrame_,
          zima_ros::TFToGeometryMsgTransform(zima_ros::MapPointToTFTransform(
              chassis_->GetLidar(chassis_->kLidar_)->GetTf()))));

  // Gazebo
  gazebo_robot_ctl_publisher_ = node.advertise<geometry_msgs::Twist>(
      "/zima_gazebo_robot_controller/cmd_vel", 1);
  gazebo_robot_odom_subscriber_ = node.subscribe(
      "/zima_gazebo_robot/gazebo_ground_truth_odom", IsRunningOnARM() ? 1 : 10,
      &ZimaGazeboNode::GazeboGroundTruthOdomCb, this);
  gazebo_robot_scan_subscriber_ =
      node.subscribe("/zima_gazebo_robot/lidar_scan", IsRunningOnARM() ? 1 : 5,
                     &ZimaGazeboNode::GazeboRobotScanCb, this);
  gazebo_clock_subscriber_ = node.subscribe(
      "/clock", IsRunningOnARM() ? 1 : 5, &ZimaGazeboNode::GazeboClockCb, this);
  gazebo_chassis_bumper_subscriber_ = node.subscribe(
      "/zima_gazebo_robot/chassis_bumper", IsRunningOnARM() ? 1 : 5,
      &ZimaGazeboNode::GazeboGeneralBumperCb, this);
  gazebo_lidar_bumper_subscriber_ = node.subscribe(
      "/zima_gazebo_robot/lidar_bumper", IsRunningOnARM() ? 1 : 5,
      &ZimaGazeboNode::GazeboGeneralBumperCb, this);
  gazebo_left_wall_sensor_subscriber_ =
      node.subscribe("/zima_gazebo_robot/left_wall_sensor_scan", 1,
                     &ZimaGazeboNode::GazeboLeftWallSensorCb, this);
  gazebo_right_wall_sensor_subscriber_ =
      node.subscribe("/zima_gazebo_robot/right_wall_sensor_scan", 1,
                     &ZimaGazeboNode::GazeboRightWallSensorCb, this);

  use_sim_time_ = true;
  sim_time_received_.store(false);
  latest_ground_truth_odom_.reset(new nav_msgs::Odometry());
  receive_msg_list_lock_.reset(new ReadWriteLock());
  last_scan_msg_.reset();
}

ZimaGazeboNode::~ZimaGazeboNode(){};

void ZimaGazeboNode::GazeboPublishChassisControl() {
  if (chassis_->IsStallTestRunning()) {
    return;
  }
  geometry_msgs::Twist twist;
  auto left_speed = chassis_->GetWheel(chassis_->kLeftWheel_)->TargetSpeed();
  auto right_speed = chassis_->GetWheel(chassis_->kRightWheel_)->TargetSpeed();

  twist.linear.x = (left_speed + right_speed) / 2;
  twist.angular.z = (right_speed - left_speed) / chassis_->GetTrackLength();
  // ZINFO << "Set Left: " << left_speed << ", right: " << right_speed;
  // ZINFO << "Set Linear: " << twist.linear.x << ", angular: " <<
  // twist.angular.z;

  gazebo_robot_ctl_publisher_.publish(twist);
  // chassis_->GetWheel(chassis_->kLeftWheel_)->SetCurrentSpeed(left_speed);
  // chassis_->GetWheel(chassis_->kRightWheel_)->SetCurrentSpeed(right_speed);
}

void ZimaGazeboNode::GazeboGroundTruthOdomCb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  WriteLocker lock(receive_msg_list_lock_);
  receive_ground_truth_odom_msg_list_.emplace_back(msg);
}

void ZimaGazeboNode::GazeboRobotScanCb(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  WriteLocker lock(receive_msg_list_lock_);
  receive_scan_msg_list_.emplace_back(msg);
}

void ZimaGazeboNode::GazeboClockCb(const rosgraph_msgs::Clock::ConstPtr &msg) {
  auto sim_time = msg->clock.toSec();
  if (sim_time_mark_ < 0 || system_time_mark_ < 0) {
    sim_time_mark_ = sim_time;
    system_time_mark_ = Time::SystemNow();
    return;
  }
  Time::UpdateFromExternalClock(sim_time);
  Time::UpdateTimeAcceleration((sim_time - sim_time_mark_) /
                               (Time::SystemNow() - system_time_mark_));
  if (use_sim_time_ && !sim_time_received_.load()) {
    sim_time_received_.store(true);
  }
}

void ZimaGazeboNode::GazeboGeneralBumperCb(
    const gazebo_msgs::ContactsState::ConstPtr &msg) {
  WriteLocker lock(receive_msg_list_lock_);
  receive_bumper_msg_list_.emplace_back(msg);
}

void ZimaGazeboNode::GazeboLeftWallSensorCb(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  static const float max_distance = 0.5;
  float distance = max_distance;
  if (!msg->ranges.empty()) {
    distance =
        msg->ranges.at(0) > max_distance ? max_distance : msg->ranges.at(0);
  } else {
    ZWARN << "Left wall sensor missing data.";
  }
  // ZINFO << "Left wall sensor: " << FloatToString(distance, 4);
  chassis_->GetWallSensor(chassis_->kLeftWallSensor_)->UpdateDistance(distance);
}

void ZimaGazeboNode::GazeboRightWallSensorCb(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  static const float max_distance = 0.5;
  float distance = max_distance;
  if (!msg->ranges.empty()) {
    distance =
        msg->ranges.at(0) > max_distance ? max_distance : msg->ranges.at(0);
  } else {
    ZWARN << "Right wall sensor missing data.";
  }
  // ZINFO << "Right wall sensor: " << FloatToString(distance, 4);
  chassis_->GetWallSensor(chassis_->kRightWallSensor_)
      ->UpdateDistance(distance);
}

bool ZimaGazeboNode::ProcessData(
    uint32_t &map_seq, std::shared_ptr<Timer> &process_map_timer) {
  bool ret = false;
  // Process ground truth odom msgs.
  {
    WriteLocker lock(receive_msg_list_lock_);
    if (!receive_ground_truth_odom_msg_list_.empty()) {
      auto msg = receive_ground_truth_odom_msg_list_.front();
      receive_ground_truth_odom_msg_list_.pop_front();
      lock.Unlock();

      auto &tf_manager = *TransformManager::Instance();
      auto recv_odom_point = zima_ros::OdometryToMapPoint(*msg);
      // ZINFO << "Receive odom: " << recv_odom_point.DebugString();
      // ZINFO << "Get Linear x: " << msg->twist.twist.linear.x
      //       << ", y: " << msg->twist.twist.linear.y
      //       << ", angular: " << msg->twist.twist.angular.z;

      auto now_time = msg->header.stamp.toSec();
      if (last_recv_odom_point_ == nullptr) {
        last_recv_odom_point_.reset(
            new TimedMapPoint(now_time, recv_odom_point));
      }
      auto time_offset = now_time - last_recv_odom_point_->GetTimestamp();

      double speed_x, speed_y;
      if (chassis_->IsStallTestRunning()) {
        auto left_speed =
            chassis_->GetWheel(chassis_->kLeftWheel_)->CurrentSpeed();
        auto right_speed =
            chassis_->GetWheel(chassis_->kRightWheel_)->CurrentSpeed();
        speed_x = (left_speed + right_speed) / 2;
        speed_y = 0;
        auto x_offset_in_robot_frame = speed_x * time_offset;
        auto y_offset_in_robot_frame = speed_y * time_offset;

        // auto twist_angular_z =
        //     (right_speed - left_speed) / chassis_->GetTrackLength();
        // auto degree_offset_in_robot_frame =
        //     RadiansToDegrees(twist_angular_z * time_offset);

        tf_manager.UpdateTransform(
            tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
            MapPoint(x_offset_in_robot_frame, y_offset_in_robot_frame, 0));
      } else {
        Transform::CoordinateTransformationAB(
            msg->twist.twist.linear.x, msg->twist.twist.linear.y, 0, 0,
            recv_odom_point.Radian(), speed_x, speed_y);
        // ZINFO << "Speed x: " << speed_x << ", y: " << speed_y;

        auto tmp = msg->twist.twist.angular.z * chassis_->GetTrackLength() / 2;
        auto left_speed = speed_x - tmp;
        auto right_speed = speed_x + tmp;
        // ZINFO << "Get Left: " << left_speed << ", right: " << right_speed;
        chassis_->GetWheel(chassis_->kLeftWheel_)->SetCurrentSpeed(left_speed);
        chassis_->GetWheel(chassis_->kRightWheel_)
            ->SetCurrentSpeed(right_speed);

        // ZINFO << "Last odom point" << last_odom_point.DebugString();
        auto x_offset_in_robot_frame = speed_x * time_offset;
        auto x_diversion_percentage = std::rand() % 20;
        x_offset_in_robot_frame *= (100.0 - x_diversion_percentage) / 100.0;
        auto y_offset_in_robot_frame = speed_y * time_offset;
        auto degree_offset_in_robot_frame =
            RadiansToDegrees(msg->twist.twist.angular.z * time_offset);
        auto degree_diversion_percentage = (std::rand() % 2) / 100.0;
        if (FLAGS_run_with_half_scan) {
          degree_diversion_percentage /= 3;
        }
        degree_offset_in_robot_frame +=
            fabs(degree_offset_in_robot_frame * degree_diversion_percentage);

        tf_manager.UpdateTransform(
            tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
            MapPoint(x_offset_in_robot_frame, y_offset_in_robot_frame,
                     degree_offset_in_robot_frame));
      }

      Transform new_odom_tf("", "");
      tf_manager.GetTransform(tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
                              new_odom_tf);

      chassis_->GetGyro(chassis_->kGyro_)
          ->SetDegree(new_odom_tf.Degree(), Time::Now());

      MapPoint new_odom_point(new_odom_tf.X(), new_odom_tf.Y(),
                              new_odom_tf.Degree());
      // ZINFO << "New odom point" << new_odom_point.DebugString();
      MergedOdomData::SPtr merged_odom_data = nullptr;
      if (chassis_->IsStallTestRunning()) {
        merged_odom_data = std::make_shared<MergedOdomData>(
            Time::Now(), new_odom_point, MapPoint(speed_x, speed_y, 0));
      } else {
        merged_odom_data = std::make_shared<MergedOdomData>(
            Time::Now(), new_odom_point,
            MapPoint(speed_x, speed_y,
                     RadiansToDegrees(msg->twist.twist.angular.z)));
      }
      chassis_->SetMergedOdomData(merged_odom_data);
      slam_wrapper_->PushMergedOdomData(merged_odom_data);
      last_recv_odom_point_->SetTimestamp(now_time);
      last_recv_odom_point_->SetX(recv_odom_point.X());
      last_recv_odom_point_->SetY(recv_odom_point.Y());
      last_recv_odom_point_->SetDegree(recv_odom_point.Degree());
      WriteLocker lock(odom_msg_access_);
      odom_msg_template_.reset(new nav_msgs::Odometry(*msg));
      latest_ground_truth_odom_.reset(new nav_msgs::Odometry(*msg));
      auto ground_truth_odom_point =
          zima_ros::OdometryToMapPoint(*latest_ground_truth_odom_);
      // ZINFO << "Receive ground truth odom: "
      //       << ground_truth_odom_point.DebugString();
      ret = true;
    }
  }

  // Process scan msgs.
  {
    WriteLocker lock(receive_msg_list_lock_);
    if (!receive_scan_msg_list_.empty()) {
      auto msg = receive_scan_msg_list_.front();
      receive_scan_msg_list_.pop_front();
      lock.Unlock();
      auto last_scan_msg_time =
          last_scan_msg_ == nullptr ? 0 : last_scan_msg_->header.stamp.toSec();
      sensor_msgs::LaserScan::Ptr _scan_msg(new sensor_msgs::LaserScan(*msg));
      _scan_msg->scan_time =
          _scan_msg->header.stamp.toSec() - last_scan_msg_time;
      // ZINFO << "Scan time: " << FloatToString(_scan_msg->scan_time, 4);
      if (enable_lidar_noise_) {
        std::random_device _random_device;
        std::default_random_engine _default_random_engine(_random_device());
        for (auto &range : _scan_msg->ranges) {
          if (range > _scan_msg->range_min && range < _scan_msg->range_max) {
            std::normal_distribution<> _normal_distribution(
                0,
                Maximum(Minimum(range, 3.0f), 0.5f) * lidar_noise_deviation_);
            range += _normal_distribution(_default_random_engine);
          }
        }
      }
      {
        WriteLocker publish_scan_msg_lock(publish_scan_msg_access_);
        publish_scan_msg_ = _scan_msg;
        if (FLAGS_run_with_half_scan) {
          for (uint i = 0; i < publish_scan_msg_->ranges.size(); i++) {
            auto degree =
                RadiansToDegrees(publish_scan_msg_->angle_min +
                                 publish_scan_msg_->angle_increment * i);
            if (degree > 90 || degree < -90) {
              publish_scan_msg_->ranges[i] =
                  std::numeric_limits<float>::infinity();
            }
          }
        }
      }
      auto new_scan = LaserScanToPointCloud(publish_scan_msg_);
      if (new_scan != nullptr) {
        chassis_->GetLidar(chassis_->kLidar_)
            ->UpdatePointCloudInLidarFrame(new_scan);
        slam_wrapper_->PushPointCloud(
            chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInLidarFrame());
      }
      last_scan_msg_ = _scan_msg;
      ret = true;
    }
  }

  // Process scan msgs.
  {
    WriteLocker lock(receive_msg_list_lock_);
    if (!receive_bumper_msg_list_.empty()) {
      auto msg = receive_bumper_msg_list_.front();
      receive_bumper_msg_list_.pop_front();
      lock.Unlock();
      MapPoints bumper_trigger_poses_on_chassis;
      for (auto &&state : msg->states) {
        for (auto &&contact_position : state.contact_positions) {
          MapPoint contact_pose(contact_position.x, contact_position.y);
          double x, y;
          auto ground_turth_odom_pose =
              zima_ros::OdometryToMapPoint(*latest_ground_truth_odom_);
          // ZINFO << "contact pose " << FloatToString(contact_position.x, 3) <<
          // ", "
          //       << FloatToString(contact_position.y, 3);
          // ZINFO << ground_turth_odom_pose.DebugString();
          Transform::CoordinateTransformationAB(
              contact_position.x, contact_position.y,
              ground_turth_odom_pose.X(), ground_turth_odom_pose.Y(),
              ground_turth_odom_pose.Radian(), x, y);
          auto trigger_pose = MapPoint(x, y);
          bumper_trigger_poses_on_chassis.emplace_back(trigger_pose);
          // ZINFO << "Trigger at " << trigger_pose.DebugString();
        }
      }

      bool center_bumper_trigger = false;
      bool left_bumper_trigger = false;
      bool right_bumper_trigger = false;
      for (auto &&pose : bumper_trigger_poses_on_chassis) {
        double pose_degree;
        if (FloatEqual(pose.X(), 0)) {
          pose_degree = 0;
        } else if (FloatEqual(pose.Y(), 0)) {
          pose_degree = pose.X() > 0 ? 90 : -90;
        } else {
          pose_degree = RadiansToDegrees(atan2(pose.Y(), pose.X()));
        }
        // ZINFO << DoubleToString(pose_degree, 3);
        if (chassis_->IsDeviceRegistered(chassis_->kCenterBumper_)) {
          auto center_bumper = chassis_->GetBumper(chassis_->kCenterBumper_);
          if (InRange(pose_degree, center_bumper->GetCoverRangeMinDegree(),
                      center_bumper->GetCoverRangeMaxDegree())) {
            center_bumper_trigger = true;
          }
        }
        auto left_bumper = chassis_->GetBumper(chassis_->kLeftBumper_);
        if (InRange(pose_degree, left_bumper->GetCoverRangeMinDegree(), 90.0)) {
          left_bumper_trigger = true;
        }
        auto right_bumper = chassis_->GetBumper(chassis_->kRightBumper_);
        if (InRange(pose_degree, -90.0,
                    right_bumper->GetCoverRangeMaxDegree())) {
          right_bumper_trigger = true;
        }
      }

      if (chassis_->IsDeviceRegistered(chassis_->kCenterBumper_)) {
        chassis_->GetBumper(chassis_->kCenterBumper_)
            ->UpdateRealtimeTriggeredState(center_bumper_trigger);
      }
      chassis_->GetBumper(chassis_->kLeftBumper_)
          ->UpdateRealtimeTriggeredState(left_bumper_trigger);
      chassis_->GetBumper(chassis_->kRightBumper_)
          ->UpdateRealtimeTriggeredState(right_bumper_trigger);

      ret = true;
    }
  }

  ret |= ZimaNode::ProcessData(map_seq, process_map_timer);
  return ret;
}

}  // namespace zima_ros
