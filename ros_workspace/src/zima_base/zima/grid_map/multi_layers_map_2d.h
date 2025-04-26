/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MULTI_LAYERS_MAP_2D_H
#define ZIMA_MULTI_LAYERS_MAP_2D_H

#include <map>
#include <memory>

#include "zima/common/debug.h"
#include "zima/grid_map/map_2d.h"

namespace zima {

class MultiLayersCharGridMap2D : public DebugBase {
 public:
  MultiLayersCharGridMap2D(const std::string name, const unsigned int& range_x,
                           const unsigned int& range_y,
                           const double& resolution);

  /**
   * @brief  Copy constructor for a dynamic map, creates a copy efficiently
   * @param map The costmap to copy
   */
  MultiLayersCharGridMap2D(const MultiLayersCharGridMap2D& map);

  ~MultiLayersCharGridMap2D() = default;

  using SPtr = std::shared_ptr<MultiLayersCharGridMap2D>;

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  MultiLayersCharGridMap2D& operator=(const MultiLayersCharGridMap2D& map);

  std::string Name() const { return name_; }
  bool CheckLayerExist(const std::string& layer_name) const;
  bool GetLayer(const std::string& layer_name,
                CharGridMap2D::SPtr& p_map) const;

  unsigned int GetRangeX() const { return range_x_; }
  unsigned int GetRangeY() const { return range_y_; }
  double GetResolution() const { return resolution_; }

  // For user using
  ReadWriteLock::SPtr GetLock() const { return access_; }

  void RemoveCopySuffix();

  std::string DebugString() const;

 protected:
  friend class MultiLayerMap2DSerializer;
  bool AddLayer(const std::string& layer_name, CharGridMap2D::SPtr p_map);

  ReadWriteLock::SPtr access_ = nullptr;
  std::string name_;
  std::map<std::string, CharGridMap2D::SPtr> char_grid_map_layers_;

  double resolution_;
  unsigned int range_x_;
  unsigned int range_y_;
};

class MultiResolutionSlamValueGridMap2D {
 public:
  MultiResolutionSlamValueGridMap2D(
      const std::string name,
      SlamValueGridMap2D::SPtr highest_resolution_slam_value_grid_map,
      const uint8_t& depth);

  /**
   * @brief  Copy constructor for a dynamic map, creates a copy efficiently
   * @param map The costmap to copy
   */
  MultiResolutionSlamValueGridMap2D(
      const MultiResolutionSlamValueGridMap2D& map);

  ~MultiResolutionSlamValueGridMap2D() = default;

  using SPtr = std::shared_ptr<MultiResolutionSlamValueGridMap2D>;

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  MultiResolutionSlamValueGridMap2D& operator=(
      const MultiResolutionSlamValueGridMap2D& map);

  bool GetLayer(const uint8_t& level, SlamValueGridMap2D::SPtr& p_map);

  // For user using
  ReadWriteLock::SPtr GetLock() const { return access_; }

  std::string DebugString() const;

 protected:
  SlamValueGridMap2D::SPtr GenerateLayer(const uint8_t& depth);

  void GenerateLayers();

  ReadWriteLock::SPtr access_ = nullptr;
  std::string name_;
  SlamValueGridMap2D::SPtr highest_resolution_slam_value_grid_map_;
  SlamValueGridMap2D::SPtr tmp_grid_map_;
  uint8_t depth_;
  std::map<uint8_t, SlamValueGridMap2D::SPtr> slam_value_grid_map_layers_;
};

class MultiLayerMap2DSerializer {
 public:
  MultiLayerMap2DSerializer() = delete;

  static ZimaProto::Map::PMultiLayerCharGridMap2D ToProto(
      const MultiLayersCharGridMap2D::SPtr& map);
  static bool FromProto(MultiLayersCharGridMap2D::SPtr& map,
                        const ZimaProto::Map::PMultiLayerCharGridMap2D& proto,
                        const unsigned int& max_width,
                        const unsigned int& max_height);
};

class MultiLayersCharGridMap2DLoader : public LocalProtoFileReader {
 public:
  MultiLayersCharGridMap2DLoader() = default;
  explicit MultiLayersCharGridMap2DLoader(const std::string& file_dir,
                                          const std::string& file_name)
      : LocalProtoFileReader(file_dir, file_name, false){};
  ~MultiLayersCharGridMap2DLoader() = default;

  bool LoadMap(MultiLayersCharGridMap2D::SPtr& map);
};

class MultiLayersCharGridMap2DWriter : public LocalProtoFileWriter {
 public:
  MultiLayersCharGridMap2DWriter() = default;
  explicit MultiLayersCharGridMap2DWriter(const std::string& file_dir,
                                          const std::string& file_name)
      : LocalProtoFileWriter(file_dir, file_name, false){};
  ~MultiLayersCharGridMap2DWriter() = default;

  bool WriteMap(const MultiLayersCharGridMap2D::SPtr& map,
                const bool& is_binary);
};

}  // namespace zima

#endif  // ZIMA_MULTI_LAYERS_MAP_2D_H
