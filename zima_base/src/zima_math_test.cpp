/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/transform.h"
#include "zima/logger/logger.h"

using namespace zima;

void TransformTest1() {
  float x = 0.027;
  float y = -0.162;
  float x_a_to_b = -0.142;
  float y_a_to_b = -0.969;
  float degree_a_to_b = 56;
  double new_x, new_y;
  Transform::CoordinateTransformationBA(
      x, y, x_a_to_b, y_a_to_b, DegreesToRadians(degree_a_to_b), new_x, new_y);
  ZINFO << "x: " << x << ", y: " << y;
  ZINFO << "new x: " << new_x << ", new y: " << new_y;
}

void TransformTest2() {
  double x = 2;
  double y = 2;
  double x_a_to_b = 1;
  double y_a_to_b = 1;
  double degree_a_to_b = 45;
  double new_x, new_y;
  Transform::CoordinateTransformationAB(
      x, y, x_a_to_b, y_a_to_b, DegreesToRadians(degree_a_to_b), new_x, new_y);
  ZINFO << "x: " << x << ", y: " << y;
  ZINFO << "new x: " << new_x << ", new y: " << new_y;
}

int main(int argc, char **argv) {
  using namespace zima;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  TransformTest1();
  TransformTest2();

  unsigned char c = 'B';
  std::string str;
  str.push_back(c);
  ZINFO << str;
  return 0;
}
