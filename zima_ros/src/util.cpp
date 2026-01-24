/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima_ros/util.h"

#include "zima/device/lidar.h"

namespace zima_ros {

using namespace zima;

MapPoint OdometryToMapPoint(const nav_msgs::Odometry& odom) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  return MapPoint(odom.pose.pose.position.x, odom.pose.pose.position.y,
                  RadiansToDegrees(tf::getYaw(q)));
}

tf2::Transform MapPointToTFTransform(const MapPoint& point) {
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(point.X(), point.Y(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, point.Radian());
  transform.setRotation(q);
  return transform;
}

MapPoint TFTransformToMapPoint(const tf::Transform& tf) {
  return MapPoint(tf.getOrigin().x(), tf.getOrigin().y(),
                  RadiansToDegrees(tf::getYaw(tf.getRotation())));
}

std::string DebugString(const tf::Transform& tf) {
  return "x: " + std::to_string(tf.getOrigin().x()) +
         ", y: " + std::to_string(tf.getOrigin().y()) + ", degree: " +
         std::to_string(RadiansToDegrees(tf::getYaw(tf.getRotation())));
}

geometry_msgs::Transform TFToGeometryMsgTransform(const tf2::Transform& tf) {
  geometry_msgs::Transform transform;
  transform.translation.x = tf.getOrigin().getX();
  transform.translation.y = tf.getOrigin().getY();
  transform.translation.z = tf.getOrigin().getZ();
  transform.rotation.x = tf.getRotation().getX();
  transform.rotation.y = tf.getRotation().getY();
  transform.rotation.z = tf.getRotation().getZ();
  transform.rotation.w = tf.getRotation().getW();
  return transform;
}

geometry_msgs::TransformStamped GeometryTransformToStampedTransform(
    const ros::Time& timestamp, const string& frame_id,
    const string& child_frame_id, const geometry_msgs::Transform& tf) {
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform = tf;
  return transform;
}

SlamValueGridMap2D::SPtr OccupancyGridToSlamValueGridMap(
    const std::string& name, const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  SlamValueGridMap2D::SPtr map(new SlamValueGridMap2D(
      name, msg->info.width, msg->info.height, msg->info.resolution));
  /* Coordinate in OccupancyGrid.
   *          x
   *    -     ^
   *    ^     |
   *    |     |
   *          |
   *  height  |
   *          |
   *    |     |
   *    v     |012345...
   *    -     |----------->y
   *
   *          |<- width ->|
   *
   * index = x * width + y
   */
  MapPoint corner_point(msg->info.origin.position.x,
                        msg->info.origin.position.y);
  WriteLocker lock(map->GetLock());
  for (auto x = 0u; x < msg->info.height; x++) {
    for (auto y = 0u; y < msg->info.width; y++) {
      auto value = msg->data.at(x * msg->info.width + y);
      MapPoint point(corner_point.X() + y * msg->info.resolution,
                     corner_point.Y() + x * msg->info.resolution);
      MapCell cell;
      map->WorldToMap(point, cell);
      map->SetValue(cell.X(), cell.Y(), value);
    }
  }

  // map->Print(__FILE__, __FUNCTION__, __LINE__);
  lock.Unlock();

  return map;
}

nav_msgs::OccupancyGrid SlamValueGridMapToOccupancyGrid(
    const std::string& frame_id, const ros::Time& time,
    const SlamValueGridMap2D::SPtr& map) {
  /* Coordinate in OccupancyGrid.
   *          x
   *    -     ^
   *    ^     |
   *    |     |
   *          |
   *  height  |
   *          |
   *    |     |
   *    v     |012345...
   *    -     |----------->y
   *
   *          |<- width ->|
   *
   * index = x * width + y
   */
  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  ReadLocker lock(map->GetLock());
  auto data_bound_min = map->GetDataBound().GetMin();
  auto data_bound_max = map->GetDataBound().GetMax();
  auto resolution = map->GetResolution();
  grid.info.origin.position.x =
      data_bound_min.X() * resolution - 0.5 * resolution;
  grid.info.origin.position.y =
      data_bound_min.Y() * resolution - 0.5 * resolution;
  // ZINFO << "pose: " << grid.info.origin.position.x << ", "
  //       << grid.info.origin.position.y;
  grid.info.height = data_bound_max.Y() - data_bound_min.Y() + 1;
  // ZINFO << "Height: " << grid.info.height;
  grid.info.width = data_bound_max.X() - data_bound_min.X() + 1;
  // ZINFO << "Width: " << grid.info.width;
  grid.info.resolution = resolution;
  // ZINFO << "resolution: " << grid.info.resolution;

  for (auto x = data_bound_min.Y(); x <= data_bound_max.Y(); x++) {
    for (auto y = data_bound_min.X(); y <= data_bound_max.X(); y++) {
      SlamValueGridMap2D::DataType value = map->GetDefaultValue();
      map->GetValue(y, x, value);
      grid.data.emplace_back(value);
    }
  }
  // ZINFO << "data size: " << grid.data.size();

  // map->Print(__FILE__, __FUNCTION__, __LINE__);

  return grid;
}

PointCloud::SPtr LaserScanToPointCloud(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  PointCloud::SPtr new_scan(
      new PointCloud(Lidar::kPointCloudInLidarFrameName_));
  WriteLocker new_scan_lock(new_scan->GetLock());
  new_scan->SetTimeStamp(scan->header.stamp.toSec());
  // new_scan->SetTimeStamp(Time::Now());
  new_scan->SetSeq(scan->header.seq);
  new_scan->SetScanTime(scan->scan_time);
  if (scan->ranges.size() != scan->intensities.size()) {
    ZWARN << "Size " << scan->ranges.size() << " vs "
          << scan->intensities.size();
    return nullptr;
  }
  auto& points = new_scan->GetPointsRef();
  points.clear();
  for (auto i = 0u; i < scan->ranges.size(); i++) {
    if (scan->ranges.at(i) > 100 || scan->ranges.at(i) < 0.01) {
      continue;
    }
    points.emplace_back(PointCloud::Point(
        scan->header.stamp.toSec() + scan->time_increment * i,
        scan->ranges.at(i),
        RadiansToDegrees(scan->angle_min + scan->angle_increment * i),
        scan->intensities.at(i)));
  }
  return new_scan;
}

}  // namespace zima_ros
