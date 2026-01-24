/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROS_NODE_H_
#define ZIMA_ROS_NODE_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "zima/algorithm/slam/slam_base.h"
#include "zima/common/lock.h"
#include "zima/common/point_cell.h"
#include "zima/common/thread.h"
#include "zima/logger/logger.h"
#include "zima/robot/chassis.h"
#include "zima/robot_data/operation_data.h"
#include "zima_ros/BlockArea.h"
#include "zima_ros/ModeSwitch.h"
#include "zima_ros/VirtualWall.h"

namespace zima_ros {

using namespace zima;

class ZimaNode {
 public:
  ZimaNode() = delete;
  explicit ZimaNode(const bool &use_simple_slam,
                    const bool &use_sim_time = false);
  ~ZimaNode();

  // InitForChassis Must run before InitForRos.
  virtual void InitForChassis();
  virtual void InitForAutoCleaning();
  virtual void InitForRos();

  void PublishCleaningInfo(SlamValueGridMap2D::SPtr map, const float &area,
                           const float &min);

  enum MapType {
    kPrintMap,
    kVirtualObsMap,
    kRoomMap,
    kUserAreaMap,
    kSlamCharMap,
  };
  void PublishMap(CharGridMap2D::SPtr map, const MapType &type,
                  const float &x_offset = 0);
  void PublishUnderLayerMap(SlamValueGridMap2D::SPtr map);
  void PublishPath(CharGridMap2D::SPtr map, const Steps &steps,
                   const float &x_offset = 0);
  void PublishPlanPath(const MapCellPath &path, const double &resolution,
                       const float &x_offset = 0);
  void PublishRobot(const MapPoint &robot_pose);
  void PublishWallSensor(const MapPoint &robot_pose);
  void PublishOdom();
  void PublishScan();
  void PublishSlamValueMap(const ros::Time &time, SlamValueGridMap2D::SPtr map);
  void PublishPointCloudInChassisFrame(
      const PointCloud::SPtr &point_cloud_in_chassis_frame);
  void PublishPointCloudInWorldFrame(
      const PointCloud::SPtr &point_cloud_in_world_frame);
  void PublishLineSegments(const LineSegments &line_segments,
                           const std::string &name_space);

  void PublishTf(const geometry_msgs::TransformStamped &tf);

  void SlamMapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void ScanCb(const sensor_msgs::LaserScan::ConstPtr &msg);

  void SetVirtualWallCb(const zima_ros::VirtualWall::ConstPtr &msg);
  void SetBlockAreaCb(const zima_ros::BlockArea::ConstPtr &msg);
  void ModeSwitchCb(const zima_ros::ModeSwitch::ConstPtr &msg);

  void ROSSpinThread(const ZimaThreadWrapper::ThreadParam &param);
  void TfThread(const ZimaThreadWrapper::ThreadParam &param);
  void PublishSensorDataThread(const ZimaThreadWrapper::ThreadParam &param);
  void DataProcessThread(const ZimaThreadWrapper::ThreadParam &param);
  void PublishForRvizThread(const ZimaThreadWrapper::ThreadParam &param);
  virtual bool ProcessData(uint32_t &map_seq,
                           std::shared_ptr<Timer> &process_map_timer);

  void Run(const ZimaThreadWrapper::ThreadParam &param);

  void Shutdown();

 protected:
  Chassis::SPtr chassis_;

  ros::NodeHandle node;

  ros::Publisher cleaning_info_marker_publisher_;
  ros::Publisher map_marker_publisher_;
  ros::Publisher path_marker_publisher_;
  ros::Publisher plan_path_marker_publisher_;
  ros::Publisher robot_marker_publisher_;
  ros::Publisher robot_sensor_marker_publisher_;
  ros::Publisher odom_publisher_;
  ros::Publisher scan_publisher_;
  ros::Publisher slam_value_map_publisher_;
  ros::Publisher point_cloud_in_chassis_frame_publisher_;
  ros::Publisher point_cloud_in_world_frame_publisher_;
  ros::Publisher line_segments_publisher_;
  ros::Subscriber mode_switch_subscriber_;
  ros::Subscriber virtual_wall_subscriber_;
  ros::Subscriber block_area_subscriber_;
  tf2_ros::StaticTransformBroadcaster static_lidar_tf_broadcaster_;
  tf2_ros::TransformBroadcaster robot_tf_broadcaster_;

  ros::Subscriber slam_map_subscriber_;

  ReadWriteLock::SPtr odom_msg_access_;
  nav_msgs::Odometry::Ptr odom_msg_template_;

  ReadWriteLock::SPtr publish_scan_msg_access_;
  sensor_msgs::LaserScan::Ptr publish_scan_msg_;

  bool use_sim_time_;
  atomic_bool sim_time_received_;

  SlamBase::SPtr slam_wrapper_;
  // Cartographer_ros
  // ros::ServiceClient cartographer_start_trajectory_client_;
  // ros::ServiceClient cartographer_stop_client_;
  // ros::ServiceClient cartographer_pause_resume_client_;
  ReadWriteLock::SPtr cache_slam_occupancy_grid_map_access_;
  nav_msgs::OccupancyGrid::ConstPtr cache_slam_occupancy_grid_map_;
  atomic_bool slam_map_received_;

  std::shared_ptr<TimedMapPoint> last_recv_odom_point_;

  ReadWriteLock::SPtr operation_data_access_;
  OperationData::SPtr operation_data_;

  ReadWriteLock::SPtr cmd_lock_;

  enum State {
    kWorking,
    kPause,
    kEnd,
  };
  State state_;

  bool use_simple_slam_;

  bool shutdown_request_;
};

}  // namespace zima_ros

#endif  // ZIMA_ROS_NODE_H_
