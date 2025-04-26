/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/slam/probability_map.h"

#include "zima/common/config.h"

namespace zima {

ProbabilityIndexGridMap2D::Config::Config() : Config(nullptr) {}

ProbabilityIndexGridMap2D::Config::Config(const JsonSPtr& json)
    : config_valid_(false) {
  // Load default setting.
  hit_probability_on_obstacle_ = 0.65;
  hit_probability_on_space_ = 1 - hit_probability_on_obstacle_;
  miss_probability_on_obstacle_ = 0.49;
  miss_probability_on_space_ = 1 - miss_probability_on_obstacle_;
  max_probability_for_obstacle_in_cell_ = 0.9;
  min_probability_for_obstacle_in_cell_ = 0.1;
  probability_count_ = 10000;

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(config);
    } else {
      config_valid_ = true;
    }
  }
}

bool ProbabilityIndexGridMap2D::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kHitProbabilityOnObstacleKey_,
                            hit_probability_on_obstacle_)) {
    ZGERROR << "Config " << kHitProbabilityOnObstacleKey_ << " not found.";
    // return false;
  }
  if (!InRange(hit_probability_on_obstacle_, 0.5f, 1.f)) {
    ZERROR << "Config " << kHitProbabilityOnObstacleKey_ << " value "
           << FloatToString(hit_probability_on_obstacle_, 4)
           << " invalid, should be 0.5 ~ 1.0.";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMissProbabilityOnObstacleKey_,
                            miss_probability_on_obstacle_)) {
    ZGERROR << "Config " << kMissProbabilityOnObstacleKey_ << " not found.";
    // return false;
  }
  if (!InRange(miss_probability_on_obstacle_, 0.f, 0.5f)) {
    ZERROR << "Config " << kMissProbabilityOnObstacleKey_ << " value "
           << FloatToString(miss_probability_on_obstacle_, 4)
           << " invalid, should be 0 ~ 0.5.";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxProbabilityForObstacleInCellKey_,
                            max_probability_for_obstacle_in_cell_)) {
    ZGERROR << "Config " << kMaxProbabilityForObstacleInCellKey_ << " not found.";
    // return false;
  }
  if (!InRange(max_probability_for_obstacle_in_cell_, 0.f, 1.f)) {
    ZERROR << "Config " << kMaxProbabilityForObstacleInCellKey_ << " value "
           << FloatToString(max_probability_for_obstacle_in_cell_, 4)
           << " invalid.";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMinProbabilityForObstacleInCellKey_,
                            min_probability_for_obstacle_in_cell_)) {
    ZGERROR << "Config " << kMinProbabilityForObstacleInCellKey_ << " not found.";
    // return false;
  }
  if (!InRange(min_probability_for_obstacle_in_cell_, 0.f, 1.f)) {
    ZERROR << "Config " << kMinProbabilityForObstacleInCellKey_ << " value "
           << FloatToString(min_probability_for_obstacle_in_cell_, 4)
           << " invalid.";
    return false;
  }

  if (min_probability_for_obstacle_in_cell_ >=
      max_probability_for_obstacle_in_cell_) {
    ZERROR << "Config " << kMinProbabilityForObstacleInCellKey_ << " value "
           << FloatToString(min_probability_for_obstacle_in_cell_, 4)
           << " is larger than " << kMaxProbabilityForObstacleInCellKey_
           << " value "
           << FloatToString(max_probability_for_obstacle_in_cell_, 4);
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kProbabilityCountKey_, probability_count_)) {
    ZGERROR << "Config " << kProbabilityCountKey_ << " not found.";
    // return false;
  }
  if (!InRange(probability_count_, 100u,
               static_cast<uint>(std::numeric_limits<int16_t>::max()))) {
    ZERROR << "Config " << kProbabilityCountKey_ << " value "
           << std::to_string(probability_count_)
           << " invalid, should be 100 ~ (int16 max).";
    return false;
  }

  return true;
}

ProbabilityIndexGridMap2D::ProbabilityIndexGridMap2D(
    const std::string& name, const uint16_t& cells_range_x,
    const uint16_t& cells_range_y, const float& resolution,
    const Config& config)
    : DynamicMap2D<ProbabilityIndex>(name, cells_range_x, cells_range_y,
                                     resolution, config.probability_count_),
      map_valid_(config.config_valid_) {
  if (!config.config_valid_) {
    ZERROR << "Config not valid.";
    return;
  }
  hit_probability_on_obstacle_ = config.hit_probability_on_obstacle_;
  hit_probability_on_space_ = config.hit_probability_on_space_;
  miss_probability_on_obstacle_ = config.miss_probability_on_obstacle_;
  miss_probability_on_space_ = config.miss_probability_on_space_;
  max_probability_for_obstacle_in_cell_ =
      config.max_probability_for_obstacle_in_cell_;
  min_probability_for_obstacle_in_cell_ =
      config.min_probability_for_obstacle_in_cell_;
  probability_count_ = config.probability_count_;
  // ZINFO << "Default value: " << std::to_string(default_value_);

  InitializeProbabilityElementVector();
}

ProbabilityIndexGridMap2D::ProbabilityIndexGridMap2D(
    const ProbabilityIndexGridMap2D& map)
    : DynamicMap2D<ProbabilityIndex>(map) {
  map_valid_ = map.map_valid_;
  hit_probability_on_obstacle_ = map.hit_probability_on_obstacle_;
  hit_probability_on_space_ = map.hit_probability_on_space_;
  miss_probability_on_obstacle_ = map.miss_probability_on_obstacle_;
  miss_probability_on_space_ = map.miss_probability_on_space_;
  max_probability_for_obstacle_in_cell_ =
      map.max_probability_for_obstacle_in_cell_;
  min_probability_for_obstacle_in_cell_ =
      map.min_probability_for_obstacle_in_cell_;
  probability_count_ = map.probability_count_;
  probability_elements_ = map.probability_elements_;
}

ProbabilityIndexGridMap2D::ProbabilityIndexGridMap2D(
    const ProbabilityIndexGridMap2D& map, const std::string& name,
    const bool& reserve)
    : DynamicMap2D<ProbabilityIndex>(map, name, reserve) {
  map_valid_ = map.map_valid_;
  hit_probability_on_obstacle_ = map.hit_probability_on_obstacle_;
  hit_probability_on_space_ = map.hit_probability_on_space_;
  miss_probability_on_obstacle_ = map.miss_probability_on_obstacle_;
  miss_probability_on_space_ = map.miss_probability_on_space_;
  max_probability_for_obstacle_in_cell_ =
      map.max_probability_for_obstacle_in_cell_;
  min_probability_for_obstacle_in_cell_ =
      map.min_probability_for_obstacle_in_cell_;
  probability_count_ = map.probability_count_;
  probability_elements_ = map.probability_elements_;
}

bool ProbabilityIndexGridMap2D::GetProbability(const int& x, const int& y,
                                               float& probability) const {
  ReadLocker lock(access_);
  DataType value;
  if (!GetValue(x, y, value)) {
    return false;
  }

  probability = ProbabilityIndexToProbability(value);
  // ZISODBG << "Value: " << std::to_string(value) << ", probability "
  //         << FloatToString(probability, 3);
  return true;
}

bool ProbabilityIndexGridMap2D::IsUnknown(const int& x, const int& y) const {
  ReadLocker lock(access_);
  DataType value;
  if (!GetValue(x, y, value)) {
    return true;
  }

  return value == static_cast<DataType>(probability_count_);
}

bool ProbabilityIndexGridMap2D::PrepareForUpdate() {
  if (!access_->IsLockedByThisThread()) {
    ZWARN << "Map: " << name_ << " is not in read/write mode.";
    return false;
  }

  for (auto x = data_bound_.GetMin().X(); x <= data_bound_.GetMax().X(); x++) {
    for (auto y = data_bound_.GetMin().Y(); y <= data_bound_.GetMax().Y();
         y++) {
      DataType value = default_value_;
      GetValue(x, y, value);
      SetValue(x, y, abs(value));
    }
  }
  return true;
}

bool ProbabilityIndexGridMap2D::UpdateForHit(const int& x, const int& y) {
  if (!access_->IsLockedByThisThread()) {
    ZWARN << "Map: " << name_ << " is not in read/write mode.";
    return false;
  }

  DataType value = default_value_;
  if (GetValue(x, y, value) && value < 0) {
    // Return if grid is updated.
    return true;
  }

  // ZINFO << "value: " << std::to_string(value);
  auto next_index =
      -1 * probability_elements_
               .at(ProbabilityIndexToProbabilityElementsIndex(value))
               .GetNextHitIndex();
  // ZINFO << "Next index: " << std::to_string(next_index);
  return SetValue(x, y, next_index);
}

bool ProbabilityIndexGridMap2D::UpdateForMiss(const int& x, const int& y) {
  if (!access_->IsLockedByThisThread()) {
    ZWARN << "Map: " << name_ << " is not in read/write mode.";
    return false;
  }

  DataType value = default_value_;
  if (GetValue(x, y, value) && value < 0) {
    // Return if grid is updated.
    return true;
  }

  // ZINFO << "value: " << std::to_string(value);
  auto next_index =
      -1 * probability_elements_
               .at(ProbabilityIndexToProbabilityElementsIndex(value))
               .GetNextMissIndex();
  // ZINFO << "Next index: " << std::to_string(next_index);
  return SetValue(x, y, next_index);
}

float ProbabilityIndexGridMap2D::GetHitProbabilityOnObstacle() const {
  ReadLocker lock(access_);
  return hit_probability_on_obstacle_;
}

float ProbabilityIndexGridMap2D::GetMissProbabilityOnObstacle() const {
  ReadLocker lock(access_);
  return miss_probability_on_obstacle_;
}

float ProbabilityIndexGridMap2D::GetMinProbabilityForObstacleInCell() const {
  ReadLocker lock(access_);
  return min_probability_for_obstacle_in_cell_;
}

float ProbabilityIndexGridMap2D::GetMaxProbabilityForObstacleInCell() const {
  ReadLocker lock(access_);
  return max_probability_for_obstacle_in_cell_;
}

float ProbabilityIndexGridMap2D::GetMediumProbabilityForObstacleInCell() const {
  ReadLocker lock(access_);
  return (max_probability_for_obstacle_in_cell_ +
          min_probability_for_obstacle_in_cell_) /
         2;
}

uint ProbabilityIndexGridMap2D::GetProbabilityCount() const {
  ReadLocker lock(access_);
  return probability_count_;
}

bool ProbabilityIndexGridMap2D::ResetProbabilityElementVector(
    const float& hit_probability_on_obstacle,
    const float& miss_probability_on_obstacle,
    const float& max_probability_for_obstacle_in_cell,
    const float& min_probability_for_obstacle_in_cell,
    const uint& probability_count) {
  if (!InRange(hit_probability_on_obstacle, 0.5f, 1.f)) {
    ZERROR << "hit_probability_on_obstacle value "
           << FloatToString(hit_probability_on_obstacle, 4)
           << " invalid, should be 0.5 ~ 1.0.";
    return false;
  }

  if (!InRange(miss_probability_on_obstacle, 0.f, 0.5f)) {
    ZERROR << "miss_probability_on_obstacle value "
           << FloatToString(miss_probability_on_obstacle, 4)
           << " invalid, should be 0 ~ 0.5.";
    return false;
  }

  if (!InRange(max_probability_for_obstacle_in_cell, 0.f, 1.f)) {
    ZERROR << "max_probability_for_obstacle_in_cell value "
           << FloatToString(max_probability_for_obstacle_in_cell, 4)
           << " invalid.";
    return false;
  }

  if (!InRange(min_probability_for_obstacle_in_cell, 0.f, 1.f)) {
    ZERROR << "min_probability_for_obstacle_in_cell value "
           << FloatToString(min_probability_for_obstacle_in_cell, 4)
           << " invalid.";
    return false;
  }

  if (min_probability_for_obstacle_in_cell >=
      max_probability_for_obstacle_in_cell) {
    ZERROR << "min_probability_for_obstacle_in_cell value "
           << FloatToString(min_probability_for_obstacle_in_cell, 4)
           << " is larger than max_probability_for_obstacle_in_cell value "
           << FloatToString(max_probability_for_obstacle_in_cell, 4);
    return false;
  }

  if (!InRange(probability_count, 100u,
               static_cast<uint>(std::numeric_limits<int16_t>::max()))) {
    ZERROR << "probability_count value " << std::to_string(probability_count)
           << " invalid, should be 100 ~ (int16 max).";
    return false;
  }

  {
    WriteLocker lock(access_);
    hit_probability_on_obstacle_ = hit_probability_on_obstacle;
    hit_probability_on_space_ = 1 - hit_probability_on_obstacle_;
    miss_probability_on_obstacle_ = miss_probability_on_obstacle;
    miss_probability_on_space_ = 1 - miss_probability_on_obstacle_;
    max_probability_for_obstacle_in_cell_ =
        max_probability_for_obstacle_in_cell;
    min_probability_for_obstacle_in_cell_ =
        min_probability_for_obstacle_in_cell;
    probability_count_ = probability_count;
  }

  InitializeProbabilityElementVector();

  return true;
}

std::vector<std::string> ProbabilityIndexGridMap2D::DebugString(
    const DynamicMapCellBound& bound, OverridePrintCellFunc func) const {
  PrintCellFunc _func = [&](const int& x, const int& y) {
    std::string msg;
    DataType value = default_value_;
    GetValue(x, y, value);
    value = abs(value);
    auto max_value = static_cast<DataType>(probability_count_ - 1);
    auto medium_value = max_value / 2;
    if (func == nullptr) {
      if (value < 0) {
        msg += ZCOLOR_RED;
        msg += "@";
        msg += ZCOLOR_NONE;
      } else if (value == default_value_) {
        msg += '.';
      } else if (value >= 0 && value <= medium_value) {
        msg += '+';
      } else if (value > medium_value && value <= max_value) {
        msg += '@';
      } else {
        msg += ZCOLOR_GREEN;
        msg += "@";
        msg += ZCOLOR_NONE;
      }
    } else {
      msg += func(value);
    }
    return msg;
  };
  return DebugStringWrapper(bound, _func);
}

std::vector<std::string> ProbabilityIndexGridMap2D::DebugString(
    OverridePrintCellFunc func) const {
  return DebugString(data_bound_, func);
}

uint32_t ProbabilityIndexGridMap2D::ProbabilityIndexToProbabilityElementsIndex(
    const ProbabilityIndex& index) const {
  int32_t _index = index;
  if (_index < 0) {
    _index = abs(_index);
  }
  _index = Clip(static_cast<int>(_index), 0,
                static_cast<int>(probability_count_));
  return _index;
}

float ProbabilityIndexGridMap2D::ProbabilityIndexToProbability(
    const ProbabilityIndex& index) const {
  return min_probability_for_obstacle_in_cell_ +
         (ProbabilityIndexToProbabilityElementsIndex(index) * 1.0 /
          (probability_count_ - 1)) *
             (max_probability_for_obstacle_in_cell_ -
              min_probability_for_obstacle_in_cell_);
}

ProbabilityIndex ProbabilityIndexGridMap2D::ProbabilityToProbabilityIndex(
    const float& probability) const {
  auto index = static_cast<ProbabilityIndex>(
      round((Clip(probability, min_probability_for_obstacle_in_cell_,
                  max_probability_for_obstacle_in_cell_) -
             min_probability_for_obstacle_in_cell_) /
            (max_probability_for_obstacle_in_cell_ -
             min_probability_for_obstacle_in_cell_) *
            (probability_count_ - 1)));
  if (index < 0) {
    ZERROR << "Index calculation failed for " << FloatToString(probability, 6);
  }
  return index;
}

float ProbabilityIndexGridMap2D::OddsToProbability(const float& odds) const {
  return odds / (1 + odds);
}

float ProbabilityIndexGridMap2D::ProbabilityToOdds(
    const float& probability) const {
  return probability / (1 - probability);
}

void ProbabilityIndexGridMap2D::InitializeProbabilityElementVector() {
  WriteLocker lock(access_);
  auto hit_update_factor =
      hit_probability_on_obstacle_ / hit_probability_on_space_;
  auto miss_update_factor =
      miss_probability_on_obstacle_ / miss_probability_on_space_;

  probability_elements_.clear();
  // If probability count is n, index should be 0 ~ (n - 1).
  probability_elements_.reserve(probability_count_ + 1);
  for (ProbabilityIndex probability_element_index = 0;
       probability_element_index < static_cast<int>(probability_count_);
       probability_element_index++) {
    // ZINFO << std::to_string(probability_element_index);
    auto probability = ProbabilityIndexToProbability(probability_element_index);
    auto odds = ProbabilityToOdds(probability);
    auto hit_update_odds = odds * hit_update_factor;
    auto hit_update_probability = Clip(OddsToProbability(hit_update_odds),
                                       min_probability_for_obstacle_in_cell_,
                                       max_probability_for_obstacle_in_cell_);
    // ZINFO << "Next probability: " << FloatToString(hit_update_probability,
    // 4);
    auto hit_update_index =
        ProbabilityToProbabilityIndex(hit_update_probability);
    auto miss_update_odds = odds * miss_update_factor;
    auto miss_update_probability = Clip(OddsToProbability(miss_update_odds),
                                        min_probability_for_obstacle_in_cell_,
                                        max_probability_for_obstacle_in_cell_);
    auto miss_update_index =
        ProbabilityToProbabilityIndex(miss_update_probability);
    probability_elements_.emplace_back(
        ProbabilityElement(probability, hit_update_index, miss_update_index));
    // ZINFO << probability_elements_.back().DebugString();
  }

  {
    auto probability = 0.5;
    auto odds = ProbabilityToOdds(probability);
    auto hit_update_odds = odds * hit_update_factor;
    auto hit_update_probability = Clip(OddsToProbability(hit_update_odds),
                                       min_probability_for_obstacle_in_cell_,
                                       max_probability_for_obstacle_in_cell_);
    // ZINFO << "Hit update probability: "
    //       << FloatToString(hit_update_probability, 4);
    auto hit_update_index =
        ProbabilityToProbabilityIndex(hit_update_probability);
    auto miss_update_odds = odds * miss_update_factor;
    auto miss_update_probability = Clip(OddsToProbability(miss_update_odds),
                                        min_probability_for_obstacle_in_cell_,
                                        max_probability_for_obstacle_in_cell_);
    auto miss_update_index =
        ProbabilityToProbabilityIndex(miss_update_probability);
    // Index n is for unknown index, make it like the medium probability one.
    probability_elements_.emplace_back(
        ProbabilityElement(probability, hit_update_index, miss_update_index));
    // ZINFO << probability_elements_.back().DebugString();
  }

  // ZINFO << "Probability_elements size " << probability_elements_.size();
}

ZimaProto::ProbabilityMap::PProbabilityIndexGridMap2D
ProbabilityIndexGridMap2DSerializer::ToProto(
    const ProbabilityIndexGridMap2D::SPtr& map) {
  ZimaProto::ProbabilityMap::PProbabilityIndexGridMap2D proto;
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return proto;
  }
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  proto.mutable_map_info()->set_name(map->Name());
  proto.mutable_map_info()->mutable_x_min_y_max_cell()->set_x(
      data_bound.GetMin().X());
  // ZINFO << "Set x "
  //       << proto.mutable_map_info()->mutable_x_min_y_max_cell()->x();
  proto.mutable_map_info()->mutable_x_min_y_max_cell()->set_y(
      data_bound.GetMax().Y());
  proto.mutable_map_info()->set_resolution(map->GetResolution());
  proto.mutable_map_info()->set_x_range(data_bound.GetMax().X() -
                                        data_bound.GetMin().X() + 1);
  proto.mutable_map_info()->set_y_range(data_bound.GetMax().Y() -
                                        data_bound.GetMin().Y() + 1);
  proto.mutable_map_info()->set_x_reserve_range(map->GetRangeX());
  proto.mutable_map_info()->set_y_reserve_range(map->GetRangeY());

  proto.set_hit_probability_on_obstacle(map->GetHitProbabilityOnObstacle());
  proto.set_miss_probability_on_obstacle(map->GetMissProbabilityOnObstacle());
  proto.set_max_probability_for_obstacle_in_cell(
      map->GetMaxProbabilityForObstacleInCell());
  proto.set_min_probability_for_obstacle_in_cell(
      map->GetMinProbabilityForObstacleInCell());
  proto.set_probability_count(map->GetProbabilityCount());

  proto.clear_data();
  for (auto y = data_bound.GetMax().Y(); y >= data_bound.GetMin().Y(); y--) {
    for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
      auto value = map->GetDefaultValue();
      map->GetValue(x, y, value);
      proto.add_data(value);
    }
  }
  return proto;
}

bool ProbabilityIndexGridMap2DSerializer::FromProto(
    ProbabilityIndexGridMap2D::SPtr& map,
    const ZimaProto::ProbabilityMap::PProbabilityIndexGridMap2D& proto) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  // Convert proto map to probability index grid map
  map->ChangeName(proto.map_info().name());
  map->ResetMap(proto.map_info().x_reserve_range(),
                proto.map_info().y_reserve_range(),
                proto.map_info().resolution(), map->GetDefaultValue());
  if (!map->ResetProbabilityElementVector(
          proto.hit_probability_on_obstacle(),
          proto.miss_probability_on_obstacle(),
          proto.max_probability_for_obstacle_in_cell(),
          proto.min_probability_for_obstacle_in_cell(),
          proto.probability_count())) {
    return false;
  }

  int map_x_min = proto.map_info().x_min_y_max_cell().x();
  int map_y_max = proto.map_info().x_min_y_max_cell().y();
  auto uint32_it = proto.data().begin();
  auto map_x = map_x_min;
  auto map_y = map_y_max;
  WriteLocker lock(map->GetLock());
  while (uint32_it != proto.data().end()) {
    if (!map->SetValue(map_x, map_y, *uint32_it)) {
      // ZINFO << "Set for " << map_x << ", " << map_y << " as: " << *uint32_it;
      ZERROR << "Error during loading value into map, set for " << map_x << ", "
             << map_y << " as: " << *uint32_it;
      return false;
    }
    map_x++;
    if (map_x - map_x_min >= proto.map_info().x_range()) {
      map_x = map_x_min;
      map_y--;
    }
    uint32_it++;
  }
  return true;
}

}  // namespace zima
