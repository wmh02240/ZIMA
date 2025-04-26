/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/line_segment_extractor.h"
#include "zima/logger/logger.h"

using namespace zima;

void LineSegmentTest1() {
  MapPoints points;
  points.emplace_back(0, 0);
  points.emplace_back(1, 0);
  points.emplace_back(2, 0);
  points.emplace_back(3, 0);
  points.emplace_back(4, 0);
  points.emplace_back(5, 0);

  Line::SPtr line;
  if (LineSegmentAlgorithm::LeastSquareFitLine(points, line)) {
    ZINFO << "Get line: " << line->DebugString(4);
  } else {
    ZERROR << "Failed to get line.";
    return;
  }

  float x1 = -3;
  float y1 = 0;
  float x2 = 0;
  float y2 = 10;

  if (line->GetY(x1, y1)) {
    MapPoint p1(x1, y1);
    ZINFO << p1.DebugString();
  } else {
    ZWARN << "Can not get Y.";
  }
  if (line->GetX(y2, x2)) {
    MapPoint p2(x2, y2);
    ZINFO << p2.DebugString();
  } else {
    ZWARN << "Can not get X.";
  }
}

void LineSegmentTest2() {
  MapPoints points;
  points.emplace_back(0, 0);
  points.emplace_back(0, 1);
  points.emplace_back(0, 2);
  points.emplace_back(0, 3);
  points.emplace_back(0, 4);
  points.emplace_back(0, 5);

  Line::SPtr line;
  if (LineSegmentAlgorithm::LeastSquareFitLine(points, line)) {
    ZINFO << "Get line: " << line->DebugString(4);
  } else {
    ZERROR << "Failed to get line.";
    return;
  }

  float x1 = -3;
  float y1 = 0;
  float x2 = 0;
  float y2 = 10;

  if (line->GetY(x1, y1)) {
    MapPoint p1(x1, y1);
    ZINFO << p1.DebugString();
  } else {
    ZWARN << "Can not get Y.";
  }
  if (line->GetX(y2, x2)) {
    MapPoint p2(x2, y2);
    ZINFO << p2.DebugString();
  } else {
    ZWARN << "Can not get X.";
  }
}

void LineSegmentTest3() {
  MapPoints points;
  points.emplace_back(0, 2);
  points.emplace_back(1, 2);
  points.emplace_back(2, 2);
  points.emplace_back(3, 2);
  points.emplace_back(4, 2);
  points.emplace_back(5, 2);

  Line::SPtr line;
  if (LineSegmentAlgorithm::LeastSquareFitLine(points, line)) {
    ZINFO << "Get line: " << line->DebugString(4);
  } else {
    ZERROR << "Failed to get line.";
    return;
  }

  float x1 = -3;
  float y1 = 0;
  float x2 = 0;
  float y2 = 10;

  if (line->GetY(x1, y1)) {
    MapPoint p1(x1, y1);
    ZINFO << p1.DebugString();
  } else {
    ZWARN << "Can not get Y.";
  }
  if (line->GetX(y2, x2)) {
    MapPoint p2(x2, y2);
    ZINFO << p2.DebugString();
  } else {
    ZWARN << "Can not get X.";
  }
}

void LineSegmentTest4() {
  MapPoints points;
  points.emplace_back(4, 0);
  points.emplace_back(4, 1);
  points.emplace_back(4, 2);
  points.emplace_back(4, 3);
  points.emplace_back(4, 4);
  points.emplace_back(4, 5);

  Line::SPtr line;
  if (LineSegmentAlgorithm::LeastSquareFitLine(points, line)) {
    ZINFO << "Get line: " << line->DebugString(4);
  } else {
    ZERROR << "Failed to get line.";
    return;
  }

  float x1 = -3;
  float y1 = 0;
  float x2 = 0;
  float y2 = 10;

  if (line->GetY(x1, y1)) {
    MapPoint p1(x1, y1);
    ZINFO << p1.DebugString();
  } else {
    ZWARN << "Can not get Y.";
  }
  if (line->GetX(y2, x2)) {
    MapPoint p2(x2, y2);
    ZINFO << p2.DebugString();
  } else {
    ZWARN << "Can not get X.";
  }
}

void LineSegmentTest5() {
  MapPoints points;
  points.emplace_back(0, 0);
  points.emplace_back(1, 1);
  points.emplace_back(2, 2);
  points.emplace_back(3, 3);
  points.emplace_back(4, 4);
  points.emplace_back(5, 5);

  Line::SPtr line;
  if (LineSegmentAlgorithm::LeastSquareFitLine(points, line)) {
    ZINFO << "Get line: " << line->DebugString(4);
  } else {
    ZERROR << "Failed to get line.";
    return;
  }

  float x1 = -3;
  float y1 = 0;
  float x2 = 0;
  float y2 = 10;

  if (line->GetY(x1, y1)) {
    MapPoint p1(x1, y1);
    ZINFO << p1.DebugString();
  } else {
    ZWARN << "Can not get Y.";
  }
  if (line->GetX(y2, x2)) {
    MapPoint p2(x2, y2);
    ZINFO << p2.DebugString();
  } else {
    ZWARN << "Can not get X.";
  }

  {
    MapPoint test_input(0, 30);
    MapPoint perpendicular_point;
    if (LineSegmentAlgorithm::PerpendicularPointOnLine(*line, test_input,
                                                       perpendicular_point)) {
      ZINFO << "Perpendicular point of " << test_input.DebugString() << " is "
            << perpendicular_point.DebugString();
    } else {
      ZWARN << "Can not get perpendicular point.";
    }
  }
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  LineSegmentTest1();
  LineSegmentTest2();
  LineSegmentTest3();
  LineSegmentTest4();
  LineSegmentTest5();

  return 0;
}
