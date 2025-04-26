/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_SIMULATOR_H
#define ZIMA_SIMULATOR_H

#include "zima/common/transform.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/robot/chassis.h"

namespace zima {

class Simulator {
 public:
  Simulator() = default;
  ~Simulator() = default;

  void Run(Chassis::SPtr chassis, TransformManager& tf_manager,
           const float& duration, NavMap::SPtr map);
};

}  // namespace zima

#endif  // ZIMA_SIMULATOR_H
