/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_OPERATION_DATA_MANAGER_H
#define ZIMA_OPERATION_DATA_MANAGER_H

#include "zima/algorithm/slam/slam_base.h"
#include "zima/robot_data/operation_data.h"

namespace zima {

class OperationDataManager {
 public:
  OperationDataManager() = delete;

  static bool CreateOperationData(
      OperationData::SPtr& operation_data, const SlamBase::SPtr& slam_wrapper,
      const OperationData::OperationType& operation_type =
          OperationData::OperationType::kAllHouseCleaning,
      const bool& use_simple_slam = false);

  static bool ReleaseOperationData(OperationData::SPtr& operation_data,
                                   const SlamBase::SPtr& slam_wrapper,
                                   const bool& use_simple_slam = false);
};

}  // namespace zima

#endif  // ZIMA_OPERATION_DATA_MANAGER_H
