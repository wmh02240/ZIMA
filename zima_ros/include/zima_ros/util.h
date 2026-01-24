/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROS_UTIL_H_
#define ZIMA_ROS_UTIL_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>

#include "zima/common/maths.h"
#include "zima/common/point_cell.h"
#include "zima/common/point_cloud.h"
#include "zima/grid_map/map_2d.h"

namespace zima_ros {

using namespace zima;

MapPoint OdometryToMapPoint(const nav_msgs::Odometry& odom);

tf2::Transform MapPointToTFTransform(const MapPoint& point);

MapPoint TFTransformToMapPoint(const tf::Transform& tf);

std::string DebugString(const tf::Transform& tf);

geometry_msgs::Transform TFToGeometryMsgTransform(const tf2::Transform& tf);

geometry_msgs::TransformStamped GeometryTransformToStampedTransform(
    const ros::Time& timestamp, const string& frame_id,
    const string& child_frame_id, const geometry_msgs::Transform& tf);

SlamValueGridMap2D::SPtr OccupancyGridToSlamValueGridMap(
    const std::string& name, const nav_msgs::OccupancyGrid::ConstPtr& msg);

nav_msgs::OccupancyGrid SlamValueGridMapToOccupancyGrid(
    const std::string& frame_id, const ros::Time& time,
    const SlamValueGridMap2D::SPtr& map);

PointCloud::SPtr LaserScanToPointCloud(
    const sensor_msgs::LaserScan::ConstPtr& scan);
}  // namespace zima_ros

#endif  // ZIMA_ROS_UTIL_H_
