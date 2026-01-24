/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/line_segment_extractor.h"

#include <algorithm>

#include "zima/common/gflags.h"
#include "zima/logger/logger.h"

namespace zima {

std::string LineSegmentAlgorithm::EvaluatedLine::DebugString(
    const uint8_t& precision) const {
  std::string msg = Line::DebugString(precision);
  msg += " mark value: " + FloatToString(mark_value_, precision);

  return msg;
}

bool LineSegmentAlgorithm::OrthogonalPointToLineDistance(const Line& line,
                                                         const MapPoint& point,
                                                         double& distance) {
  if (!line.IsValid()) {
    return false;
  }
  distance =
      fabs(line.GetA() * point.X() + line.GetB() * point.Y() + line.GetC()) /
      sqrt(Square(line.GetA()) + Square(line.GetB()));
  return true;
}

bool LineSegmentAlgorithm::PerpendicularPointOnLine(
    const Line& line, const MapPoint& input_point,
    MapPoint& perpendicular_point) {
  if (!line.IsValid()) {
    return false;
  }
  auto x = (Square(line.GetB()) * input_point.X() -
            line.GetA() * line.GetB() * input_point.Y() -
            line.GetA() * line.GetC()) /
           (Square(line.GetA()) + Square(line.GetB()));
  perpendicular_point.SetX(x);
  auto y = (Square(line.GetA()) * input_point.Y() -
            line.GetA() * line.GetB() * input_point.X() -
            line.GetB() * line.GetC()) /
           (Square(line.GetA()) + Square(line.GetB()));
  perpendicular_point.SetY(y);

  return true;
}

bool LineSegmentAlgorithm::LeastSquareFitLine(const MapPoints& points,
                                              Line::SPtr& result_line) {
  return LeastSquareFitLine(points.begin(), points.end(), result_line);
}

bool LineSegmentAlgorithm::LeastSquareFitLine(
    const MapPoints::const_iterator& start_it,
    const MapPoints::const_iterator& end_it, Line::SPtr& result_line) {
  auto n = distance(start_it, end_it);
  if (n < 1) {
    ZWARN << "Insufficient iterator distance: " << n;
    return false;
  }

  double x_sum = 0;
  double y_sum = 0;
  double x_square_sum = 0;
  double x_times_y_sum = 0;
  for (auto it = start_it; it != end_it; it++) {
    x_sum += it->X();
    y_sum += it->Y();
    x_square_sum += Square(it->X());
    x_times_y_sum += it->X() * it->Y();
  }

  // ZINFO << "x_sum: " << FloatToString(x_sum, 3);
  // ZINFO << "y_sum: " << FloatToString(y_sum, 3);
  // ZINFO << "x_square_sum: " << FloatToString(x_square_sum, 3);
  // ZINFO << "x_times_y_sum: " << FloatToString(x_times_y_sum, 3);

  auto m = (x_square_sum * n - Square(x_sum));
  // ZINFO << "m: " << FloatToString(m, 3);
  if (DoubleEqual(m, 0)) {
    auto a = 1;
    auto b = 0;
    auto c = -1 * x_sum / n;
    result_line.reset(new Line(a, b, c));
    // ZINFO;
  } else {
    auto k = (x_times_y_sum * n - x_sum * y_sum) / m;
    // ZINFO << "k: " << FloatToString(k, 3);
    auto a = k;
    auto b = -1;
    auto c = y_sum / n - a * x_sum / n;
    result_line.reset(new Line(a, b, c));
    // ZINFO;
  }

  return true;
}

bool LineSegmentAlgorithm::AveragePointToLineDistance(
    const MapPoints& points, const Line& line, double& average_distance) {
  if (points.empty()) {
    return false;
  }
  return AveragePointToLineDistance(points.cbegin(), points.cend(), line,
                                    average_distance);
}

bool LineSegmentAlgorithm::AveragePointToLineDistance(
    const MapPoints::const_iterator& start_it,
    const MapPoints::const_iterator& end_it, const Line& line,
    double& average_distance) {
  if (!line.IsValid()) {
    return false;
  }
  auto n = distance(start_it, end_it);
  if (n < 1) {
    ZWARN << "Insufficient iterator distance: " << n;
    return false;
  }
  double distance_sum = 0;

  for (auto it = start_it; it != end_it; it++) {
    double distance = 0;
    if (OrthogonalPointToLineDistance(line, *it, distance)) {
      distance_sum += distance;
    } else {
      ZWARN << "Get distance failed.";
    }
  }

  average_distance = distance_sum / (n + 1);

  return true;
}

bool LineSegmentAlgorithm::MostValuedLineDegree(const LineSegments& lines,
                                                double& degree) {
  if (lines.empty()) {
    return false;
  }

  static auto convert_degree_to_0_90_range =
      [](const double& input_degree) -> double {
    auto _degree = NormalizeDegree(input_degree);
    while (_degree < 0) {
      _degree += 90;
    }
    while (_degree > 90) {
      _degree -= 90;
    }
    return _degree;
  };

  using Element = std::pair<uint8_t /*degree*/, double /*score*/>;
  std::vector<Element> element_vector;

  for (auto&& line : lines) {
    auto score = line.Length();
    int16_t index;
    if (DoubleEqual(line.GetB(), 0)) {
      index = 90;
    } else {
      auto degree = convert_degree_to_0_90_range(
          RadiansToDegrees(atan(-1 * line.GetA() / line.GetB())));
      // ZINFO << "Degree: " << FloatToString(degree, 2);
      index = round(degree);
      if (index < 0 || index >= 180) {
        ZERROR << "Invalid index: " << index
               << " line: " << line.DebugString(4);
        continue;
      }
      auto it = std::find_if(element_vector.begin(), element_vector.end(),
                             [index](const Element& element) -> bool {
                               return element.first == index;
                             });
      if (it == element_vector.end()) {
        element_vector.emplace_back(Element(index, score));
      } else {
        it->second += score;
      }
    }
  }

  std::sort(element_vector.begin(), element_vector.end(),
            [](const Element& a, const Element& b) -> bool {
              return a.second > b.second;
            });

  // const uint8_t kDebugPrintMaxCount = 3;
  // auto debug_print_count = 0;
  // auto element_it = element_vector.begin();
  // while (++debug_print_count < kDebugPrintMaxCount) {
  //   if (element_it == element_vector.end()) {
  //     break;
  //   }
  //   ZINFO << "Degree: " << static_cast<int>(element_it->first)
  //         << ", score: " << FloatToString(element_it->second, 2);
  //   element_it++;
  // }

  if (!element_vector.empty()) {
    degree = element_vector.begin()->first;
    if (degree > 45) {
      degree -= 90;
    }
    // ZINFO << "Most valued degree: " << FloatToString(degree, 1);
  }

  return true;
}

const uint16_t PointCloudLineSegmentExtractor::kPointCloudPointsCountLimit_ = 4;

PointClouds PointCloudLineSegmentExtractor::SeparatePointCloudByPointDistance(
    const PointCloud& point_cloud) {
  PointClouds result;
  ReadLocker lock(point_cloud.GetLock());
  if (point_cloud.Size() < kPointCloudPointsCountLimit_) {
    ZWARN << "Insufficient source point cloud size: " << point_cloud.Size()
          << ".";
    return result;
  }
  PointCloud::SPtr tmp_point_cloud = nullptr;
  uint16_t index = 0;
  auto& source_points = point_cloud.GetPointsConstRef();
  auto point_it1 = source_points.begin();
  // ZINFO << "First point: " << point_it1->DebugString();
  auto point_it2 = point_it1 + 1;

  const auto kPointsDistanceTolerance = 4.0f;
  const double kPointDistanceLimitMin =
      point_cloud.GetFilterDistance() * kPointsDistanceTolerance;
  // ZWARN << "Limit: " << FloatToString(kPointDistanceLimitMin, 3);
  const auto kDegreeDiffLimit = 5.0f;

  auto add_point_cloud_if_valid = [&]() -> void {
    ReadLocker lock(tmp_point_cloud->GetLock());
    if (tmp_point_cloud->Size() >= kPointCloudPointsCountLimit_) {
      result.emplace_back(tmp_point_cloud);
      index++;
      //   ZINFO << "Add point cloud.";
      // } else {
      //   ZWARN << "Insufficient point cloud point size: "
      //         << tmp_point_cloud->Size() << ".";
    }
    tmp_point_cloud.reset();
  };

  auto found_break_point = [&](const PointCloud::Point& point1,
                               const PointCloud::Point& point2) -> bool {
    auto min_distance = std::min(point1.Distance(), point2.Distance());

    auto points_degree_diff =
        NormalizeDegree(point1.Degree() - point2.Degree());
    auto distance = point1.ToMapPoint().Distance(point2.ToMapPoint());

    // Point distance means distance from point cloud point to origin point,
    // points distance means distance from point cloud point to another point
    // cloud point.
    auto points_distance_estimate_from_point_distance =
        min_distance * fabs(sin(DegreesToRadians(points_degree_diff)));
    auto points_distance_limit_estimate_from_point_distance =
        min_distance * fabs(sin(DegreesToRadians(kDegreeDiffLimit)));

    const auto point_distance_limit =
        std::max(kPointDistanceLimitMin,
                 std::min(points_distance_estimate_from_point_distance,
                          points_distance_limit_estimate_from_point_distance));

    // ZWARN << "Point 1 " << point1.DebugString();
    // ZWARN << "Point 2 " << point2.DebugString();
    // ZWARN << "Distance: " << FloatToString(distance, 4)
    //       << " limit: " << FloatToString(point_distance_limit, 4)
    //       << " point distance limit: "
    //       << FloatToString(points_distance_estimate_from_point_distance, 4);

    return distance > point_distance_limit;
  };

  while (true) {
    if (tmp_point_cloud == nullptr) {
      tmp_point_cloud.reset(
          new PointCloud("Point cloud slice " + std::to_string(index)));
      WriteLocker lock(tmp_point_cloud->GetLock());
      tmp_point_cloud->GetPointsRef().emplace_back(*point_it1);
    }

    if (point_it2 == source_points.end()) {
      // Check if last point cloud link to first point cloud.
      if (!result.empty()) {
        WriteLocker lock(result.front()->GetLock());
        auto& first_point_cloud_points = result.front()->GetPointsRef();
        if (FloatEqual(first_point_cloud_points.front().Degree(),
                       source_points.begin()->Degree())) {
          point_it2 = source_points.begin();
          if (!found_break_point(*point_it1, *point_it2)) {
            ReadLocker lock(tmp_point_cloud->GetLock());
            auto& tmp_point_cloud_points = tmp_point_cloud->GetPointsConstRef();
            first_point_cloud_points.insert(first_point_cloud_points.begin(),
                                            tmp_point_cloud_points.begin(),
                                            tmp_point_cloud_points.end());
            // ZINFO << "Fit " << tmp_point_cloud_points.size()
            //       << " points of last point cloud into first point cloud.";
          }
        }
      }
      break;
    }

    if (found_break_point(*point_it1, *point_it2)) {
      add_point_cloud_if_valid();
    } else {
      WriteLocker lock(tmp_point_cloud->GetLock());
      tmp_point_cloud->GetPointsRef().emplace_back(*point_it2);
      // ZINFO << "Emplace point.";
    }

    point_it1 = point_it2;
    point_it2++;
  }

  return result;
}

bool PointCloudLineSegmentExtractor::EstimatePointCloudPointOnLine(
    const Line& line, const float& point_cloud_degree,
    MapPoint& estimate_point) {
  if (!line.IsValid()) {
    return false;
  }

  auto radian = DegreesToRadians(point_cloud_degree);
  auto x = -1 * line.GetC() * cos(radian) /
           (line.GetA() * cos(radian) + line.GetB());
  estimate_point.SetX(x);
  auto y = x * tan(radian);
  estimate_point.SetY(y);
  return true;
}

LineSegments PointCloudLineSegmentExtractor::ExtractLineSegments(
    const PointCloud& point_cloud) {
  // Learn from seed-region-growing method
  LineSegments line_segments;

  MapPoints all_points;
  ReadLocker lock(point_cloud.GetLock());
  auto& point_cloud_points = point_cloud.GetPointsConstRef();
  for (auto&& point_cloud_point : point_cloud_points) {
    all_points.emplace_back(point_cloud_point.ToMapPoint());
  }
  lock.Unlock();

  MapPoints::const_iterator find_seed_start_it = all_points.begin();
  MapPoints::const_iterator seed_start_it = find_seed_start_it;
  MapPoints::const_iterator seed_end_it = find_seed_start_it;
  PointCloudLineSegmentExtractor::SeedRegionGrowingConfig config;

  LineSegments seed_line_segments;
  while (true) {
    if (seed_end_it == all_points.end()) {
      break;
    }
    LineSegment::SPtr seed_line_segment = nullptr;
    if (!FindSeedLineSegment(all_points, find_seed_start_it, seed_start_it,
                             seed_end_it, seed_line_segment, config)) {
      // ZWARN << "Fail to find seed line.";
      break;
    } else {
      // ZINFO << "Get seed line segment: " <<
      // seed_line_segment->DebugString(4);
      seed_line_segments.emplace_back(*seed_line_segment);
      auto seed_grow_end_it = seed_end_it;
      if (GrowFromSeedLineSegment(all_points, seed_start_it, seed_end_it,
                                  seed_grow_end_it, seed_line_segment,
                                  config)) {
        find_seed_start_it = seed_grow_end_it;
      } else {
        // Grow failed.
        find_seed_start_it = seed_end_it;
        // ZWARN << "Grow line from seed failed.";
      }

      if (seed_line_segment->Length() > config.grow_line_length_threshold_) {
        line_segments.emplace_back(*seed_line_segment);
      }
    }
  }
  return line_segments;
}

bool PointCloudLineSegmentExtractor::GetMostValuedLineDegree(
    const PointCloud::SPtr& source_point_cloud, double& degree) {
  if (source_point_cloud == nullptr) {
    ZWARN << "Source point cloud invalid.";
    return false;
  }

  auto separate_point_clouds =
      PointCloudLineSegmentExtractor::SeparatePointCloudByPointDistance(
          *source_point_cloud);
  // ZINFO << "Saperate to " << separate_point_clouds.size()
  //       << " point clouds";
  LineSegments saperate_point_cloud_group;
  LineSegments line_segments;
  for (auto&& point_cloud : separate_point_clouds) {
    ReadLocker lock(point_cloud->GetLock());
    if (point_cloud->Size() <
        PointCloudLineSegmentExtractor::kPointCloudPointsCountLimit_) {
      // ZWARN << "Point cloud size: " << point_cloud->Size();
      continue;
    }
    auto& point_cloud_points = point_cloud->GetPointsConstRef();
    saperate_point_cloud_group.emplace_back(LineSegment(
        point_cloud_points.front().X(), point_cloud_points.front().Y(),
        point_cloud_points.back().X(), point_cloud_points.back().Y()));
    // ZINFO << "Add point cloud group: "
    //       << saperate_point_cloud_group.back().DebugString(2);

    auto _line_segments =
        PointCloudLineSegmentExtractor::ExtractLineSegments(*point_cloud);
    line_segments.insert(line_segments.end(), _line_segments.begin(),
                         _line_segments.end());
  }
  ZGINFO << "Get " << line_segments.size() << " lines.";
  if (LineSegmentAlgorithm::MostValuedLineDegree(line_segments, degree)) {
    ZGINFO << "Most valued degree: " << FloatToString(degree, 1);
    return true;
  }

  ZGWARN << "Failed to get most valued degree.";
  return false;
}

PointCloudLineSegmentExtractor::SeedRegionGrowingConfig::
    SeedRegionGrowingConfig() {
  seed_line_average_deviation_limit_ = 0.07;
  seed_line_improve_percentage_threshold_ = 0.2;
  grow_line_length_threshold_ = 0.4;
}

bool PointCloudLineSegmentExtractor::FindSeedLineSegment(
    const MapPoints& points, const MapPoints::const_iterator& start_point_it,
    MapPoints::const_iterator& seed_start_point_it,
    MapPoints::const_iterator& seed_end_point_it,
    LineSegment::SPtr& line_segment, const SeedRegionGrowingConfig& config) {
  // Find seed line segment
  static auto try_to_fit_a_short_line =
      [](const MapPoints::const_iterator& start_it,
         const MapPoints::const_iterator& end_it,
         LineSegmentAlgorithm::AverageDeviationEvaluatedLine::SPtr& result_line)
      -> bool {
    if (distance(start_it, end_it) < kPointCloudPointsCountLimit_ - 1) {
      ZWARN << distance(start_it, end_it);
      return false;
    }
    Line::SPtr line = nullptr;
    if (LineSegmentAlgorithm::LeastSquareFitLine(start_it, end_it, line)) {
      double average_deviation = 0;
      if (LineSegmentAlgorithm::AveragePointToLineDistance(
              start_it, end_it, *line, average_deviation)) {
        result_line.reset(
            new LineSegmentAlgorithm::AverageDeviationEvaluatedLine(
                line->GetA(), line->GetB(), line->GetC(), average_deviation));
        return true;
      } else {
        ZWARN;
      }
    } else {
      ZWARN;
    }
    return false;
  };

  if (points.size() < kPointCloudPointsCountLimit_) {
    ZWARN << "Insufficient point cloud size: " << points.size() << ".";
    return false;
  }

  auto start_it = start_point_it;
  auto end_it = start_it;
  for (auto i = 0; i < kPointCloudPointsCountLimit_ + 1; i++) {
    if (end_it == points.cend()) {
      // ZWARN << "Insufficient point cloud left.";
      return false;
    }
    end_it = start_it + i;
  }

  auto saved_start_it = start_it;
  auto saved_end_it = end_it;
  LineSegmentAlgorithm::AverageDeviationEvaluatedLine::SPtr result_line =
      nullptr;
  auto step = kPointCloudPointsCountLimit_;
  while (true) {
    if (end_it == points.cend()) {
      break;
    }
    if (step > kPointCloudPointsCountLimit_) {
      break;
    }
    if (result_line != nullptr) {
      step++;
    }

    LineSegmentAlgorithm::AverageDeviationEvaluatedLine::SPtr tmp_line =
        nullptr;
    if (try_to_fit_a_short_line(start_it, end_it, tmp_line)) {
      // ZINFO << "Get temp line: " << tmp_line->DebugString(4);
      if (tmp_line->GetMarkValue() <
          config.seed_line_average_deviation_limit_) {
        if (result_line == nullptr) {
          // Initialize for line.
          result_line = tmp_line;
          saved_start_it = start_it;
          saved_end_it = end_it;
        } else if (tmp_line->GetMarkValue() < result_line->GetMarkValue() &&
                   ((result_line->GetMarkValue() - tmp_line->GetMarkValue()) /
                        result_line->GetMarkValue() >
                    config.seed_line_improve_percentage_threshold_)) {
          // Update for more than 20% better line.
          result_line = tmp_line;
          saved_start_it = start_it;
          saved_end_it = end_it;
        } else {
          // Stop for worse line.
          break;
        }
      }
    }
    start_it++;
    end_it++;
  }

  if (result_line == nullptr) {
    // Can not find proper seed line.
    // ZWARN << "Can not find proper seed line.";
    return false;
  }

  MapPoint start_point, end_point;
  auto ret_begin_point = LineSegmentAlgorithm::PerpendicularPointOnLine(
      *result_line, *saved_start_it, start_point);
  auto ret_end_point = LineSegmentAlgorithm::PerpendicularPointOnLine(
      *result_line, *saved_end_it, end_point);
  if (!ret_begin_point || !ret_end_point) {
    ZWARN << "Failed to find point " << ret_begin_point << ", "
          << ret_end_point;
    return false;
  }

  seed_start_point_it = saved_start_it;
  seed_end_point_it = saved_end_it;
  line_segment.reset(new LineSegment(start_point.X(), start_point.Y(),
                                     end_point.X(), end_point.Y()));

  // ZINFO << "Valid: " << line_segment->IsValid();
  // ZINFO << "Found seed line.";
  return true;
}

bool PointCloudLineSegmentExtractor::GrowFromSeedLineSegment(
    const MapPoints& points,
    const MapPoints::const_iterator& seed_start_point_it,
    const MapPoints::const_iterator& seed_end_point_it,
    MapPoints::const_iterator& grow_end_point_it,
    LineSegment::SPtr& seed_line_segment,
    const SeedRegionGrowingConfig& config) {
  auto grow_start_point_it = seed_end_point_it + 1;
  grow_end_point_it = grow_start_point_it;

  LineSegment::SPtr new_line_segment = nullptr;

  while (true) {
    if (grow_end_point_it == points.end()) {
      break;
    }
    double distance = 0;
    bool ret = false;
    if (new_line_segment == nullptr) {
      ret = LineSegmentAlgorithm::OrthogonalPointToLineDistance(
          *seed_line_segment, *grow_end_point_it, distance);
    } else {
      ret = LineSegmentAlgorithm::OrthogonalPointToLineDistance(
          *new_line_segment, *grow_end_point_it, distance);
    }
    if (!ret) {
      ZWARN;
      break;
    }

    bool stop_growing = true;
    if (distance < config.seed_line_average_deviation_limit_) {
      // Try to fit a new line.
      Line::SPtr tmp_line = nullptr;
      if (LineSegmentAlgorithm::LeastSquareFitLine(
              seed_start_point_it, grow_end_point_it, tmp_line)) {
        double tmp_average_deviation = 0;
        if (LineSegmentAlgorithm::AveragePointToLineDistance(
                seed_start_point_it, grow_end_point_it, *tmp_line,
                tmp_average_deviation)) {
          if (tmp_average_deviation <
              config.seed_line_average_deviation_limit_) {
            MapPoint start_point, end_point;
            auto ret_begin_point =
                LineSegmentAlgorithm::PerpendicularPointOnLine(
                    *tmp_line, *seed_start_point_it, start_point);
            auto ret_end_point = LineSegmentAlgorithm::PerpendicularPointOnLine(
                *tmp_line, *grow_end_point_it, end_point);
            if (!ret_begin_point || !ret_end_point) {
              ZWARN << "Failed to find point " << ret_begin_point << ", "
                    << ret_end_point;
            } else {
              new_line_segment.reset(
                  new LineSegment(start_point.X(), start_point.Y(),
                                  end_point.X(), end_point.Y()));
              stop_growing = false;
            }
          } else {
            // New line deviation is too high.
          }
        } else {
          ZWARN << "Failed to calculate average deviation line.";
        }
      } else {
        ZWARN << "Failed to Fit line.";
      }
    } else {
      // Point too far from line.
    }

    if (stop_growing) {
      break;
    }

    grow_end_point_it++;
  }

  if (new_line_segment == nullptr) {
    // ZWARN << "Grow line from seed failed.";
    return false;
  }

  seed_line_segment = new_line_segment;
  return true;
}

}  // namespace zima
