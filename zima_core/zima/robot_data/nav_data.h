/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_NAV_DATA_H
#define ZIMA_NAV_DATA_H

#include <map>

#include "zima/algorithm/slam/probability_map.h"
#include "zima/common/macro.h"
#include "zima/common/point_cell.h"
#include "zima/common/steps_recorder.h"
#include "zima/common/time.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/proto/nav_data.pb.h"

namespace zima {

class RoomInfo {
 public:
  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kRoomInfoConfigKey_;

    static const std::string kMaxSectionWidthKey_;
    float max_section_width_;
  };

  RoomInfo();
  RoomInfo(const RoomInfo& ref);
  RoomInfo(const CharGridMap2D::DataType& room_index,
           const DynamicMapCellBound room_bound,
           const MapCell& section_base_cell,
           const unsigned int& section_x_range,
           const unsigned int& section_y_range);
  ~RoomInfo() = default;
  using SPtr = std::shared_ptr<RoomInfo>;

  static void Initialize();

  bool UpdateRoomIndex(const CharGridMap2D::DataType& index);
  CharGridMap2D::DataType GetRoomIndex() const {
    ReadLocker lock(access_);
    return room_index_;
  }
  DECLARE_DATA_GET_SET(DynamicMapCellBound, RoomBound)
  DECLARE_DATA_GET_SET(MapCell, SectionBaseCell)
  DECLARE_DATA_GET_SET(uint16_t, SectionXRange)
  DECLARE_DATA_GET_SET(uint16_t, SectionYRange)

  DynamicMapCellBound GetSectionBound(const MapCell& cell);

  void SetCleaned(const bool& value) { cleaned_.store(value); }
  bool IsCleaned() const { return cleaned_.load(); }

  std::string DebugString() const;

  static float kMaxSectionWidth_;

 private:
  ReadWriteLock::SPtr access_;
  CharGridMap2D::DataType room_index_;
  DynamicMapCellBound room_bound_;
  MapCell section_base_cell_;
  uint16_t section_x_range_;
  uint16_t section_y_range_;
  std::atomic_bool cleaned_;
};

using RoomsInfo = std::map<CharGridMap2D::DataType, RoomInfo::SPtr>;

class NavDataSerializer;
class NavData {
 public:
  NavData();
  NavData(const bool& quiet);
  NavData(const NavData& ref);
  ~NavData();
  using SPtr = std::shared_ptr<NavData>;
  using SCPtr = std::shared_ptr<const NavData>;

  enum UserBlockType {
    kStrictBlock,
    kAvoidWaterBlock,
    kCarpetBlock,
  };

  class VirtualWall {
   public:
    VirtualWall() = delete;
    VirtualWall(const MapPoint& point1, const MapPoint& point2,
                const bool& enable = true,
                const UserBlockType& type = UserBlockType::kStrictBlock);

    using SPtr = std::shared_ptr<VirtualWall>;

    bool IsValid() const { return valid_; }
    bool IsEnable() const { return enable_; }
    CharGridMap2D::DataType GetMapValue() const { return map_value_; }
    MapCells ToMapCells(const CharGridMap2D::SPtr& map);
    std::string DebugString(const CharGridMap2D::SPtr& map) const;

   private:
    friend NavDataSerializer;
    bool valid_;
    bool enable_;
    UserBlockType type_;
    CharGridMap2D::DataType map_value_;
    MapPoints points_;
  };

  using VirtualWalls = std::map<uint8_t, VirtualWall::SPtr>;

  class BlockArea {
   public:
    BlockArea() = delete;
    BlockArea(const MapPoint& point1, const MapPoint& point2,
              const MapPoint& point3, const MapPoint& point4,
              const bool& enable = true,
              const UserBlockType& type = UserBlockType::kStrictBlock);

    using SPtr = std::shared_ptr<BlockArea>;

    bool IsValid() const { return valid_; }
    bool IsEnable() const { return enable_; }
    CharGridMap2D::DataType GetMapValue() const { return map_value_; }
    MapCells ToMapCells(const CharGridMap2D::SPtr& map);
    std::string DebugString(const CharGridMap2D::SPtr& map) const;

   private:
    friend NavDataSerializer;
    bool valid_;
    bool enable_;
    UserBlockType type_;
    CharGridMap2D::DataType map_value_;
    MapPoints points_;
  };

  using BlockAreas = std::map<uint8_t, BlockArea::SPtr>;

  DECLARE_DATA_GET_SET(uint32_t, Index)
  NavMap::SPtr GetNavMapRef() const;
  NavMap::SCPtr GetNavMapConstRef() const;
  SlamValueGridMap2D::SPtr GetRawSlamValueGridMap2DRef() const;
  void UpdateRawSlamValueGridMap2D(const SlamValueGridMap2D::SPtr& value);
  SlamValueGridMap2D::SPtr GetOptimizedSlamValueGridMap2DRef() const;
  void UpdateOptimizedSlamValueGridMap2D(const SlamValueGridMap2D::SPtr& value);
  ProbabilityIndexGridMap2D::SPtr GetProbabilityIndexGridMap2DRef() const;
  void UpdateProbabilityIndexGridMap2D(
      const ProbabilityIndexGridMap2D::SPtr& value);
  DECLARE_DATA_GET_SET(std::string, SlamMapFileName)
  RoomInfo::SPtr GetCurrentRoomInfoRef() const;
  DECLARE_DATA_GET_SET(RoomsInfo, RoomsInfo)
  bool AddVirtualWall(const VirtualWall::SPtr& virtual_wall);
  bool UpdateVirtualWall(const uint8_t& index,
                         const VirtualWall::SPtr& virtual_wall);
  bool RemoveVirtualWall(const uint8_t& index);
  bool GetVirtualWall(const uint8_t& index,
                      VirtualWall::SPtr& virtual_wall) const;
  VirtualWalls GetAllVirtualWall() const;
  bool AddBlockArea(const BlockArea::SPtr& block_area);
  bool UpdateBlockArea(const uint8_t& index, const BlockArea::SPtr& block_area);
  bool RemoveBlockArea(const uint8_t& index);
  bool GetBlockArea(const uint8_t& index, BlockArea::SPtr& block_area) const;
  BlockAreas GetAllBlockArea() const;

  RoomsInfo GetSelectedRoomsInfo() const;
  bool UpdateSelectedRoomsInfo(const RoomsInfo& selected_rooms);
  void ResumeSelectedRoomsInfo();
  std::string DebugAvailableRoomsInfo();
  std::string DebugUserSelectRoomsInfo();

  void UpdateUserBlockMapInNavMap();
  std::string DebugUserBlocksInfo();

  void UpdateUserSelectAreaInNavMap(
      const CharGridMap2D::SPtr& new_user_select_area_map);
  void ResumeUserSelectAreaInNavMap();

  static const std::string kRawSlamMapName_;
  static const std::string kOptimizedSlamMapName_;
  static const uint8_t kMaxVirtualWallSize_;
  static const uint8_t kMaxBlockAreaSize_;

 protected:
  friend class NavDataSerializer;
  uint32_t index_;  // Use time stamp as index.
  NavMap::SPtr nav_map_;
  SlamValueGridMap2D::SPtr raw_slam_map_;
  SlamValueGridMap2D::SPtr optimized_slam_map_;
  ProbabilityIndexGridMap2D::SPtr probability_grid_map_;
  std::string slam_map_filename_;  // For cartographer
  RoomInfo::SPtr current_room_info_;
  RoomsInfo rooms_info_;
  RoomsInfo selected_rooms_info_;
  VirtualWalls virtual_walls_;
  BlockAreas block_areas_;
  ReadWriteLock::SPtr access_;
};

class NavDataSerializer {
 public:
  NavDataSerializer() = delete;

  static ZimaProto::NavData::PRoomInfo ToProto(const RoomInfo::SPtr& room_info);
  static bool FromProto(RoomInfo::SPtr& room_info,
                        const ZimaProto::NavData::PRoomInfo& proto);

  static ZimaProto::NavData::PVirtualWall ToProto(
      const NavData::VirtualWall::SPtr& virtual_wall);
  static bool FromProto(NavData::VirtualWall::SPtr& virtual_wall,
                        const ZimaProto::NavData::PVirtualWall& proto);

  static ZimaProto::NavData::PBlockArea ToProto(
      const NavData::BlockArea::SPtr& block_area);
  static bool FromProto(NavData::BlockArea::SPtr& block_area,
                        const ZimaProto::NavData::PBlockArea& proto);

  static ZimaProto::NavData::PNavData ToProto(const NavData::SPtr& data);
  static bool FromProto(NavData::SPtr& data,
                        const ZimaProto::NavData::PNavData& proto);
};

class NavDataLoader : public LocalProtoFileReader {
 public:
  NavDataLoader() = default;
  explicit NavDataLoader(const std::string& file_dir,
                         const std::string& file_name)
      : LocalProtoFileReader(file_dir, file_name){};
  ~NavDataLoader() = default;

  bool LoadData(NavData::SPtr& data, const bool& quiet = false);
  bool LoadData(NavData::VirtualWall::SPtr& data);
  bool LoadData(NavData::BlockArea::SPtr& data);
};

class NavDataWriter : public LocalProtoFileWriter {
 public:
  NavDataWriter() = default;
  explicit NavDataWriter(const std::string& file_dir,
                         const std::string& file_name)
      : LocalProtoFileWriter(file_dir, file_name){};
  ~NavDataWriter() = default;

  bool WriteData(const NavData::SPtr& data, const bool& is_binary);
  bool WriteData(const NavData::VirtualWall::SPtr& data, const bool& is_binary);
  bool WriteData(const NavData::BlockArea::SPtr& data, const bool& is_binary);
};

}  // namespace zima

#endif  // ZIMA_NAV_DATA_H
