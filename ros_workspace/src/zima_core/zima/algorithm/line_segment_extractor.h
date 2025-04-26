/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_LINE_SEGMENT_EXTRACTOR_H
#define ZIMA_LINE_SEGMENT_EXTRACTOR_H

#include <deque>

#include "zima/common/point_cloud.h"

namespace zima {

class LineSegmentAlgorithm {
 public:
  class EvaluatedLine : public Line {
   public:
    EvaluatedLine() = delete;
    EvaluatedLine(const float& a, const float& b, const float& c,
                  const float& mark_value)
        : Line(a, b, c), mark_value_(mark_value) {}
    ~EvaluatedLine() = default;

    float GetMarkValue() const { return mark_value_; }

    std::string DebugString(const uint8_t& precision) const;

   protected:
    const float mark_value_;
  };

  class AverageDeviationEvaluatedLine : public EvaluatedLine {
   public:
    AverageDeviationEvaluatedLine() = delete;
    AverageDeviationEvaluatedLine(const float& a, const float& b,
                                  const float& c, const float& mark_value)
        : EvaluatedLine(a, b, c, mark_value) {}
    ~AverageDeviationEvaluatedLine() = default;
    using SPtr = std::shared_ptr<AverageDeviationEvaluatedLine>;
  };

  LineSegmentAlgorithm() = default;
  ~LineSegmentAlgorithm() = default;

  static bool OrthogonalPointToLineDistance(const Line& line,
                                            const MapPoint& point,
                                            double& distance);
  static bool PerpendicularPointOnLine(const Line& line,
                                       const MapPoint& input_point,
                                       MapPoint& perpendicular_point);

  static bool LeastSquareFitLine(const MapPoints& points,
                                 Line::SPtr& result_line);
  static bool LeastSquareFitLine(const MapPoints::const_iterator& start_it,
                                 const MapPoints::const_iterator& end_it,
                                 Line::SPtr& result_line);

  static bool AveragePointToLineDistance(const MapPoints& points,
                                         const Line& line,
                                         double& average_distance);
  static bool AveragePointToLineDistance(
      const MapPoints::const_iterator& start_it,
      const MapPoints::const_iterator& end_it, const Line& line,
      double& average_distance);

  static bool MostValuedLineDegree(const LineSegments& lines, double& degree);
};

class PointCloudLineSegmentExtractor {
 public:
  PointCloudLineSegmentExtractor() = default;
  ~PointCloudLineSegmentExtractor() = default;

  static PointClouds SeparatePointCloudByPointDistance(
      const PointCloud& point_cloud);

  static bool EstimatePointCloudPointOnLine(const Line& line,
                                            const float& point_cloud_degree,
                                            MapPoint& estimate_point);

  static LineSegments ExtractLineSegments(const PointCloud& point_cloud);

  static bool GetMostValuedLineDegree(
      const PointCloud::SPtr& source_point_cloud, double& degree);

  // Learn from seed-region-growing method
  class SeedRegionGrowingConfig {
    public:
     SeedRegionGrowingConfig();
     ~SeedRegionGrowingConfig() = default;

     float seed_line_average_deviation_limit_;
     float seed_line_improve_percentage_threshold_;
     float grow_line_length_threshold_;
  };

  static bool FindSeedLineSegment(
      const MapPoints& points, const MapPoints::const_iterator& start_point_it,
      MapPoints::const_iterator& seed_start_point_it,
      MapPoints::const_iterator& seed_end_point_it,
      LineSegment::SPtr& line_segment, const SeedRegionGrowingConfig& config);

  static bool GrowFromSeedLineSegment(
      const MapPoints& points,
      const MapPoints::const_iterator& seed_start_point_it,
      const MapPoints::const_iterator& seed_end_point_it,
      MapPoints::const_iterator& grow_end_point_it,
      LineSegment::SPtr& seed_line_segment,
      const SeedRegionGrowingConfig& config);

  static const uint16_t kPointCloudPointsCountLimit_;
};

}  // namespace zima

#endif  // ZIMA_LINE_SEGMENT_EXTRACTOR_H
