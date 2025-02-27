/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/common/point_cell.h"
#include "zima/common/point_cloud.h"
#include "zima/grid_map/map_2d.h"
#include "zima/logger/logger.h"

using namespace zima;

void PointCloudRotateTest() {
  auto file_dir = "../zima/proto/";
  auto file_name = "gazebo_room.pb.txt";
  SlamValueGridMap2DLoader loader(file_dir, file_name);
  SlamValueGridMap2D::SPtr test_map(new SlamValueGridMap2D(
      "test", 50, 50, 0.1));
  test_map->Print(__FILE__, __FUNCTION__, __LINE__);
  if (loader.LoadMap(test_map)) {
    test_map->Print(__FILE__, __FUNCTION__, __LINE__);
  } else {
    ZERROR;
  }

  PointCloud::SPtr new_scan(new PointCloud("test"));
  WriteLocker scan_lock(new_scan->GetLock());
  new_scan->SetTimeStamp(0);
  new_scan->SetSeq(0);
  auto &points = new_scan->GetPointsRef();
  points.clear();
  points.emplace_back(PointCloud::Point(MapPoint(1, 1), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, 0.8), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, 0.6), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, 0.4), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, 0.2), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, 0), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, -0.2), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, -0.4), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, -0.6), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, -0.8), 100));
  points.emplace_back(PointCloud::Point(MapPoint(1, -1), 100));
  scan_lock.Unlock();

  PointCloudMatcher::Config config;
  PointCloudMatcher matcher(config);
  MapPoint init_pose(0.3, 0.2, 12);
  DynamicMapPointBound bound_in_map_frame(test_map->GetDataPointBound());
  PointCloudMatcher::BABSearchParameter parameter(
      test_map, init_pose, bound_in_map_frame, 180, 1, 7, 0.6);
  PointCloudMatcher::MatchResult result;
  ZINFO;
  // matcher.BABSearch(test_map, new_scan, parameter, result);
  matcher.BoostBABSearch(test_map, new_scan, parameter, result);
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  PointCloudRotateTest();

  return 0;
}
