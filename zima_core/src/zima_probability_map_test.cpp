/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/logger/logger.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/algorithm/slam/probability_map.h"

using namespace zima;

void ProbabilityMapProbabilityElementsTest() {
  ProbabilityIndexGridMap2D::Config config;
  ProbabilityIndexGridMap2D::SPtr probability_map(new ProbabilityIndexGridMap2D(
      "test map", 30, 30, NavMap::GetResolution(), config));
  ProbabilityIndex index = 0;
  auto probability_elements = probability_map->GetProbabilityElements();
  auto i = 0;
  for (; i < 300; i++) {
    auto element = probability_elements.at(index);
    ZINFO << element.DebugString();
    index = element.GetNextHitIndex();
    if (FloatEqual(element.GetProbability(),
                   config.max_probability_for_obstacle_in_cell_)) {
      break;
    }
  }
  ZERROR << "It takes " << i << " steps.";
  index = probability_elements.size() - 2;
  i = 0;
  for (; i < 300; i++) {
    auto element = probability_elements.at(index);
    ZINFO << element.DebugString();
    index = element.GetNextMissIndex();
    if (FloatEqual(element.GetProbability(),
                   config.min_probability_for_obstacle_in_cell_)) {
      break;
    }
  }
  ZERROR << "It takes " << i << " steps.";
}

void ProbabilityMapTest() {
  ProbabilityIndexGridMap2D::Config config;
  ProbabilityIndexGridMap2D::SPtr probability_map(new ProbabilityIndexGridMap2D(
      "test map", 30, 30, NavMap::GetResolution(), config));

  WriteLocker lock(probability_map->GetLock());
  for (auto x = 0; x < 5; x++) {
    for (auto y = 0; y < 5; y++) {
      probability_map->UpdateForHit(x, y);
    }
  }

  for (auto x = 0; x > -5; x--) {
    for (auto y = 0; y > -5; y--) {
      probability_map->UpdateForMiss(x, y);
    }
  }

  probability_map->Print(__FILE__, __FUNCTION__, __LINE__);

  for (auto count = 0; count <= 5; count++) {
    probability_map->PrepareForUpdate();
    for (auto x = 0; x < 5; x++) {
      for (auto y = 0; y < 5; y++) {
        probability_map->UpdateForMiss(x, y);
      }
    }

    for (auto x = 0; x > -5; x--) {
      for (auto y = 0; y > -5; y--) {
        probability_map->UpdateForHit(x, y);
      }
    }
  }

  probability_map->Print(__FILE__, __FUNCTION__, __LINE__);
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // ProbabilityMapProbabilityElementsTest();
  ProbabilityMapTest();

  return 0;
}
