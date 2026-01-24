/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_NAV_MAP_2D_H
#define ZIMA_NAV_MAP_2D_H

#include "zima/common/gflags.h"
#include "zima/common/marker_points.h"
#include "zima/common/transform.h"
#include "zima/grid_map/multi_layers_map_2d.h"

namespace zima {

class NavMap : public MultiLayersCharGridMap2D {
 public:
  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kResolutionKey_;
    float resolution_;
    static const std::string kMaxWidthKey_;
    float max_width_;
    static const std::string kMaxHeightKey_;
    float max_height_;
    static const std::string kMaxCleanWidthKey_;
    float max_clean_width_;
    static const std::string kMaxCleanHeightKey_;
    float max_clean_height_;
  };

  NavMap();
  NavMap(const NavMap& map);
  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  NavMap& operator=(const NavMap& map);

  ~NavMap();

  using SPtr = std::shared_ptr<NavMap>;
  using SCPtr = std::shared_ptr<const NavMap>;

  static void LoadConfig();

  static double GetResolution();

  void RefreshLayerPtrs();
  CharGridMap2D::SPtr GetPrintLayer() const;
  CharGridMap2D::SPtr GetFootStepLayer() const;
  CharGridMap2D::SPtr GetSensorLayer() const;
  CharGridMap2D::SPtr GetSlamLayer() const;
  CharGridMap2D::SPtr GetRoomLayer() const;
  CharGridMap2D::SPtr GetUserBlockLayer() const;
  CharGridMap2D::SPtr GetUserSelectAreaLayer() const;
  bool ChangeSlamLayer(const CharGridMap2D::SPtr& new_char_slam_map);
  bool ChangeRoomLayer(const CharGridMap2D::SPtr& new_room_map);
  bool ChangeUserSelectAreaLayer(
      const CharGridMap2D::SPtr& new_user_select_area_map);

  MapCells GetRobotCoverCells(const MapPoint& pose) const;
  MapCells GetFootstepCoverCells(const MapPoint& pose,
                                 const MapPoints& addition_point = {}) const;

  void MarkForCleaningSteps(const TransformManager& tf_manager,
                            const Steps& path,
                            const DynamicMapCellBound& section_bound_,
                            const bool& mark_for_edge = false);
  void MarkForPassingThroughSteps(const TransformManager& tf_manager,
                                  const Steps& path,
                                  const DynamicMapCellBound& section_bound_);
  void MarkForAlongSideSteps(const TransformManager& tf_manager,
                             const Steps& path, const bool& on_left,
                             const DynamicMapCellBound& section_bound_);

  void DebugMarkForSensor(const MapCell& cell,
                          const CharGridMap2D::DataType& value) {
    if (IsDebugFuncEnabled()) {
      WriteLocker print_lock(print_layer_->GetLock());
      WriteLocker sensor_lock(sensor_layer_->GetLock());
      MarkForSensor(cell, value);
    }
  }

  void ClearRobotCoverCells(const MapPoint& pose);

  bool IsClearedInSensorMap(const MapCell& cell) const;

  float GetStepAreaSize() const;

  void ClearSteps();

  DynamicMapCellBound GetMaxCleanBound(
      const MapCell::SPtr& curr_pose = nullptr) const;

  static int16_t GetMaxCleanCellWidth() { return max_clean_cell_width_; }
  static int16_t GetMaxCleanCellHeight() { return max_clean_cell_height_; }

  static bool is_config_loaded_;
  // Robot width is devided into 5 cells.
  static double resolution_;
  static const int kRobotCellWidth_;
  static const int kRobotCellWidth_2_;
  static const int kRobotMarkWidth_2_;

  // Print map is just for range below.
  static const std::string kPrintMapName_;
  // ======Print map range=======
  static const std::string kFootStepMapName_;
  static const std::string kSensorMapName_;
  static const std::string kUserBlockMapName_;
  // ====Print map range end=====

  static const std::string kSlamMapName_;
  static const std::string kRoomMapName_;
  static const std::string kUserSelectAreaMapName_;

  static const CharGridMap2D::DataType kUnknown_;
  static const CharGridMap2D::DataType kFootStep_;

  // Sensor accurate obstacles
  static const CharGridMap2D::DataType kBumper_;
  static const CharGridMap2D::DataType kCliff_;
  static const CharGridMap2D::DataType kWall_;

  // Slam semantic value
  static const CharGridMap2D::DataType kSlamWall_;
  static const CharGridMap2D::DataType kSlamFloor_;

  // Markers
  static const CharGridMap2D::DataType kVirtualWall_;
  static const CharGridMap2D::DataType kStrictBlockArea_;
  static const CharGridMap2D::DataType kAvoidWaterBlockArea_;
  static const CharGridMap2D::DataType kCarpetBlockArea_;
  static const CharGridMap2D::DataType kCharger_;
  static const CharGridMap2D::DataType kUserSelected_;
  static const CharGridMap2D::DataType kUserUnSelected_;

  // Room id
#define DeclareRoomValue(name) \
  static const CharGridMap2D::DataType kRoom##name##_;

  DeclareRoomValue(A)
  DeclareRoomValue(B)
  DeclareRoomValue(C)
  DeclareRoomValue(D)
  DeclareRoomValue(E)
  DeclareRoomValue(F)
  DeclareRoomValue(G)
  DeclareRoomValue(H)
  DeclareRoomValue(I)
  DeclareRoomValue(J)
  DeclareRoomValue(K)
  DeclareRoomValue(L)
  DeclareRoomValue(M)
  DeclareRoomValue(N)
  DeclareRoomValue(O)
  DeclareRoomValue(P)

  static const uint8_t kMaxRoomCount_;

  // Debug marker
  static const CharGridMap2D::DataType kPathStart_;
  static const CharGridMap2D::DataType kPath_;

  static float max_width_;
  static float max_height_;

 private:
  void MarkForFootStep(const MapCell& cell,
                       const CharGridMap2D::DataType& value);
  void MarkForEmptyFootStep(const MapCell& cell);
  bool ClearSensor(const MapCell& cell);
  void MarkForSensor(const MapCell& cell, const CharGridMap2D::DataType& value);

  CharGridMap2D::SPtr print_layer_;
  CharGridMap2D::SPtr footstep_layer_;
  CharGridMap2D::SPtr sensor_layer_;
  CharGridMap2D::SPtr slam_layer_;
  CharGridMap2D::SPtr room_layer_;
  CharGridMap2D::SPtr user_block_layer_;
  CharGridMap2D::SPtr user_select_area_layer_;

  MapPoints robot_cover_points_;
  MapPoints foot_print_points_;

  MapCells map_near_4_cells_;

  static int16_t max_clean_cell_width_;
  static int16_t max_clean_cell_height_;
};

class NavMapLoader : public MultiLayersCharGridMap2DLoader {
 public:
  NavMapLoader() = default;
  explicit NavMapLoader(const std::string& file_dir,
                        const std::string& file_name)
      : MultiLayersCharGridMap2DLoader(file_dir, file_name){};
  ~NavMapLoader() = default;

  bool LoadMap(NavMap::SPtr& map);
};

class NavMapWriter : public MultiLayersCharGridMap2DWriter {
 public:
  NavMapWriter() = default;
  explicit NavMapWriter(const std::string& file_dir,
                        const std::string& file_name)
      : MultiLayersCharGridMap2DWriter(file_dir, file_name){};
  ~NavMapWriter() = default;

  bool WriteMap(const NavMap::SPtr& map, const bool& is_binary);
};

}  // namespace zima

#endif  // ZIMA_NAV_MAP_2D_H
