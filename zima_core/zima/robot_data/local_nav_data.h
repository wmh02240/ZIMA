/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_LOCAL_NAV_DATA_H
#define ZIMA_LOCAL_NAV_DATA_H

#include <atomic>

#include "zima/common/macro.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class LocalNavData {
 public:
  LocalNavData() = delete;
  LocalNavData(const NavData::SPtr& nav_data, const std::string& localize_path,
               const bool& localized = false);
  ~LocalNavData() = default;

  using SPtr = std::shared_ptr<LocalNavData>;

  bool IsValid() const;
  bool Localize();
  static void ClearLocalization(const std::string& file_names,
                                atomic_bool& localized_flag);

  bool LoadFromLocal(const std::string& name);

  NavData::SCPtr GetConstNavDataDataRef();
  NavData::SPtr GetNavDataCopyData();

  static const std::string kSuffix_;
  static const std::string kSlamFileSuffix_;

 private:
  ReadWriteLock::SPtr access_;
  NavData::SPtr nav_data_;
  atomic_bool valid_;
  atomic_bool localized_;
  const std::string kLocalizePath_;
};

class LocalNavDataManager {
  DECLARE_SINGLETON(LocalNavDataManager)
 public:
  ~LocalNavDataManager();

  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kLocalNavDataPathKey_;
    std::string local_nav_data_path_;
    static const std::string kLocalNavDataCountLimitKey_;
    uint local_nav_data_count_limit_;
  };

  using LocalNavDatas = std::deque<LocalNavData::SPtr>;
  using LocalNavDataIndexs = std::deque<uint32_t>;

  bool AddNewNavData(const NavData::SPtr& nav_data);
  bool UpdateNavData(const NavData::SPtr& nav_data);
  bool RemoveNavData(const uint32_t& index);
  bool SelectNavData(const uint32_t& index);
  bool UnSelectNavData();
  LocalNavData::SPtr GetNavData(const uint32_t& index);
  uint32_t GetSelectedNavDataIndex() const;
  std::vector<uint32_t> GetNavDataIndexList() const;
  std::string GetNavDataPath() const { return localize_path_; }
  uint16_t GetNavDataCountLimit() const { return nav_data_count_limit_; }

  static const std::string kSelectedNavDataSuffix_;

 private:
  void LoadNavDatas(const std::vector<std::string>& names);

  ReadWriteLock::SPtr access_;
  std::string localize_path_;
  uint16_t nav_data_count_limit_;
  LocalNavDatas local_nav_datas_;
  LocalNavDataIndexs local_nav_data_indexs_;
  uint32_t selected_nav_data_index_;

  const bool kPreloadLocalNavDataIntoMemory_;
};

}  // namespace zima

#endif  // ZIMA_LOCAL_NAV_DATA_H
