/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/logger/logger.h"
#include "zima/algorithm/pose_interpolator.h"

using namespace zima;

void PoseInterpolatorTest1() {
  PoseInterpolator::Config config;
  PoseInterpolator pose_interpolator(config);
  pose_interpolator.AddData(
      std::make_shared<MergedOdomData>(1, MapPoint(-1, -1, 0)));
  pose_interpolator.AddData(
      std::make_shared<MergedOdomData>(1.5, MapPoint(1, 1, 60)));

  auto test_time = 0.3;
  ZINFO << DoubleToString(test_time, 1) << " IsInterpolatable: "
        << pose_interpolator.IsInterpolatable(test_time);
  test_time = 1.2;
  ZINFO << DoubleToString(test_time, 1) << " IsInterpolatable: "
        << pose_interpolator.IsInterpolatable(test_time);
  test_time = 1.8;
  ZINFO << DoubleToString(test_time, 1) << " IsInterpolatable: "
        << pose_interpolator.IsInterpolatable(test_time);

  std::deque<MergedOdomData::SPtr> data_deque;
  test_time = 1.1;
  while (test_time <= 1.5) {
    auto interpolate_pose = pose_interpolator.InterpolatePose(test_time);
    if (interpolate_pose != nullptr) {
      data_deque.emplace_back(interpolate_pose);
    } else {
      ZERROR << "Failed for " << DoubleToString(test_time, 2);
    }
    test_time += 0.1;
  }

  for (auto &&data : data_deque) {
    ZINFO << data->DebugString();
  }

  data_deque.clear();
  test_time = 1.6;
  while (test_time <= 2) {
    auto predict_pose = pose_interpolator.PredictPose(test_time);
    if (predict_pose != nullptr) {
      data_deque.emplace_back(predict_pose);
    } else {
      ZERROR << "Failed for " << DoubleToString(test_time, 2);
    }
    test_time += 0.1;
  }

  for (auto &&data : data_deque) {
    ZINFO << data->DebugString();
  }
}

void PoseInterpolatorTest2() {
  PoseInterpolator::Config config;
  PoseInterpolator pose_interpolator(config);
  pose_interpolator.AddData(std::make_shared<MergedOdomData>(
      1, MapPoint(-1, -1, 0), MapPoint(0, 0, 0)));
  pose_interpolator.AddData(std::make_shared<MergedOdomData>(
      1.5, MapPoint(1, 1, 60), MapPoint(1, 1, 60)));

  auto test_time = 0.3;
  ZINFO << DoubleToString(test_time, 1) << " IsInterpolatable: "
        << pose_interpolator.IsInterpolatable(test_time);
  test_time = 1.2;
  ZINFO << DoubleToString(test_time, 1) << " IsInterpolatable: "
        << pose_interpolator.IsInterpolatable(test_time);
  test_time = 1.8;
  ZINFO << DoubleToString(test_time, 1) << " IsInterpolatable: "
        << pose_interpolator.IsInterpolatable(test_time);

  std::deque<MergedOdomData::SPtr> data_deque;
  test_time = 1.1;
  while (test_time <= 1.5) {
    auto interpolate_pose = pose_interpolator.InterpolatePose(test_time);
    if (interpolate_pose != nullptr) {
      data_deque.emplace_back(interpolate_pose);
    } else {
      ZERROR << "Failed for " << DoubleToString(test_time, 2);
    }
    test_time += 0.1;
  }

  for (auto &&data : data_deque) {
    ZINFO << data->DebugString();
  }

  data_deque.clear();
  test_time = 1.6;
  while (test_time <= 2) {
    auto predict_pose = pose_interpolator.PredictPose(test_time);
    if (predict_pose != nullptr) {
      data_deque.emplace_back(predict_pose);
    } else {
      ZERROR << "Failed for " << DoubleToString(test_time, 2);
    }
    test_time += 0.1;
  }

  for (auto &&data : data_deque) {
    ZINFO << data->DebugString();
  }
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  PoseInterpolatorTest1();
  ZERROR;
  PoseInterpolatorTest2();

  return 0;
}
