/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot_data/local_nav_data.h"

#include <algorithm>

#include "zima/common/config.h"
#include "zima/hal/system/cmd_line.h"

namespace zima {

const std::string LocalNavData::kSuffix_ = ".local_nav_data.pbstream";
const std::string LocalNavData::kSlamFileSuffix_ =
    ".local_nav_data_slam.pbstream";
const std::string LocalNavDataManager::kSelectedNavDataSuffix_ = ".selected";

LocalNavData::LocalNavData(const NavData::SPtr& nav_data,
                           const std::string& localize_path,
                           const bool& localized)
    : access_(std::make_shared<ReadWriteLock>()),
      localized_(localized),
      kLocalizePath_(localize_path) {
  if (nav_data == nullptr) {
    nav_data_ = nullptr;
    valid_.store(false);
    // ZINFO << "Create empty nav_data(localized " << localized_.load() << ").";
  } else {
    nav_data_ = std::make_shared<NavData>(*nav_data);
    nav_data_->GetNavMapRef()->RemoveCopySuffix();
    nav_data_->GetRawSlamValueGridMap2DRef()->RemoveCopySuffix();
    nav_data_->GetOptimizedSlamValueGridMap2DRef()->RemoveCopySuffix();
    // Do not need step info.
    nav_data_->GetNavMapRef()->ClearSteps();
    valid_.store(true);
    ZINFO << "Create nav_data(localized " << localized_.load() << " valid "
          << valid_.load()
          << ") index: " << static_cast<int>(nav_data_->GetIndex());
  }
}

bool LocalNavData::IsValid() const { return valid_.load(); }

bool LocalNavData::Localize() {
  if (!valid_.load()) {
    ZWARN << "Nav data was not valid.";
    return false;
  }

  if (localized_.load()) {
    ZWARN << "Nav data " << Time::DebugString(nav_data_->GetIndex())
          << " has been localized.";
    return true;
  }

  ReadLocker lock(access_);

  const auto index = nav_data_->GetIndex();
  const auto file_name = std::to_string(index) + kSuffix_;
  const auto link_file_name =
      Time::DebugString(nav_data_->GetIndex()) + kSuffix_;
  const auto slam_file_name = std::to_string(index) + kSlamFileSuffix_;

  NavDataWriter writter(kLocalizePath_, file_name);
  if (!writter.WriteData(nav_data_, true)) {
    ClearLocalization(kLocalizePath_ + file_name + " " + kLocalizePath_ +
                          link_file_name + " " + kLocalizePath_ +
                          slam_file_name,
                      localized_);
    ZWARN << "Nav data " << Time::DebugString(nav_data_->GetIndex())
          << " localization failed.";
    return false;
  }

  if (!FileSystemHelper::IsFileOrDirectoryExists(kLocalizePath_ +
                                                 link_file_name)) {
    std::string cmd = "ln -s " + kLocalizePath_ + file_name + " " +
                      kLocalizePath_ + link_file_name;
    if (ZimaRunCommand(cmd) != 0) {
      ClearLocalization(kLocalizePath_ + file_name + " " + kLocalizePath_ +
                            link_file_name + " " + kLocalizePath_ +
                            slam_file_name,
                        localized_);
      ZWARN << "Nav data " << Time::DebugString(nav_data_->GetIndex())
            << " localization failed.";
      return false;
    }
  }

  localized_.store(true);

  return true;
}

void LocalNavData::ClearLocalization(const std::string& file_names,
                                        atomic_bool& localized_flag) {
  std::string cmd = "rm -rf " + file_names;
  if (ZimaRunCommand(cmd) != 0) {
    ZWARN << "Clear for file \"" << file_names << "\" failed.";
    return;
  }

  localized_flag.store(false);
  ZINFO << "Clear for file \"" << file_names << "\" succeed.";
  return;
}

bool LocalNavData::LoadFromLocal(const std::string& name) {
  if (valid_.load()) {
    ZWARN << "This nav_data is already loaded.";
    return false;
  }

  WriteLocker lock(access_);
  NavDataLoader loader(kLocalizePath_, name);
  nav_data_ = std::make_shared<NavData>(true);
  if (!loader.LoadData(nav_data_, true)) {
    ZWARN << "Load for nav_data file " << kLocalizePath_ << name << " failed.";
    return false;
  }
  valid_.store(true);
  localized_.store(true);
  // ZINFO << "Load nav_data(localized " << localized_.load() << ")";
  ZGINFO << "Load nav_data(" << name << ")";

  return true;
}

NavData::SCPtr LocalNavData::GetConstNavDataDataRef() {
  ReadLocker lock(access_);
  return nav_data_;
}

NavData::SPtr LocalNavData::GetNavDataCopyData() {
  ReadLocker lock(access_);
  NavData::SPtr ptr = std::make_shared<NavData>(*nav_data_);
  ptr->GetNavMapRef()->RemoveCopySuffix();
  ptr->GetRawSlamValueGridMap2DRef()->RemoveCopySuffix();
  ptr->GetOptimizedSlamValueGridMap2DRef()->RemoveCopySuffix();

  return ptr;
}

LocalNavDataManager::Config::Config() : Config(nullptr){};

LocalNavDataManager::Config::Config(const JsonSPtr& json) : config_valid_(false) {
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
      ZERROR;
    }
  }
};

bool LocalNavDataManager::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetString(*json, kLocalNavDataPathKey_, local_nav_data_path_)) {
    ZERROR << "Config " << kLocalNavDataPathKey_ << " not found.";
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kLocalNavDataCountLimitKey_,
                           local_nav_data_count_limit_)) {
    ZERROR << "Config " << kLocalNavDataCountLimitKey_ << " not found.";
    return false;
  }

  return true;
}

LocalNavDataManager::LocalNavDataManager()
    : access_(std::make_shared<ReadWriteLock>()),
      kPreloadLocalNavDataIntoMemory_(false) {
  Config config;
  localize_path_ = config.config_valid_ ? config.local_nav_data_path_
                                        : "/tmp/zima_local_nav_data/";
  nav_data_count_limit_ =
      config.config_valid_ ? config.local_nav_data_count_limit_ : 3;

  if (!FileSystemHelper::IsFileOrDirectoryExists(localize_path_)) {
    FileSystemHelper::CreateDirectory(localize_path_);
  }
  std::vector<std::string> file_names;
  if (!FileSystemHelper::GetFilesNamesInDirectory(localize_path_, file_names)) {
    ZERROR << "Directory unavailable.";
  } else {
    std::vector<std::string> nav_data_names;
    for (auto&& file_name : file_names) {
      if (StringEndsWith(file_name, LocalNavData::kSuffix_)) {
        nav_data_names.emplace_back(file_name);
      }
    }
    LoadNavDatas(nav_data_names);
  }

  selected_nav_data_index_ = 0;

  std::vector<std::string> link_file_names;
  if (!FileSystemHelper::GetLinkFilesNamesInDirectory(localize_path_,
                                                      link_file_names)) {
    ZERROR << "Directory unavailable.";
  } else {
    // for (auto&& name : link_file_names) {
    //   ZINFO << name;
    // }
    auto selected_file_it =
        find_if(link_file_names.begin(), link_file_names.end(),
                [](const std::string& file_name) -> bool {
                  return StringEndsWith(file_name, kSelectedNavDataSuffix_);
                });

    if (selected_file_it != link_file_names.end()) {
      auto selected_file = *selected_file_it;
      for (auto len = 0u; len < kSelectedNavDataSuffix_.size(); len++) {
        selected_file.pop_back();
      }
      if (StringEndsWith(selected_file, LocalNavData::kSuffix_)) {
        for (auto len = 0u; len < LocalNavData::kSuffix_.size(); len++) {
          selected_file.pop_back();
        }
        selected_nav_data_index_ =
            static_cast<uint32_t>(atoi(selected_file.c_str()));
        ZGINFO << "Load selected nav_data as "
              << selected_nav_data_index_;
      } else {
        ZERROR << "File name invalid: " << *selected_file_it;
      }
    } else {
      ZGINFO << "No nav_data selected.";
    }
  }
}

bool LocalNavDataManager::AddNewNavData(
    const NavData::SPtr& nav_data) {
  ZINFO << "Add new nav_data.";
  LocalNavData::SPtr new_nav_data =
      std::make_shared<LocalNavData>(nav_data, localize_path_);
  if (!new_nav_data->IsValid()) {
    ZERROR << "Failed for empty nav_data.";
    return false;
  }

  if (!new_nav_data->Localize()) {
    ZERROR << "Failed for localization failure.";
    return false;
  }

  WriteLocker lock(access_);
  if (kPreloadLocalNavDataIntoMemory_) {
    std::sort(local_nav_datas_.begin(), local_nav_datas_.end(),
              [](const LocalNavData::SPtr& a, const LocalNavData::SPtr& b) {
                return a->GetConstNavDataDataRef()->GetIndex() <
                       b->GetConstNavDataDataRef()->GetIndex();
              });
    while (local_nav_datas_.size() >= nav_data_count_limit_) {
      for (auto local_nav_data_it = local_nav_datas_.begin();
           local_nav_data_it != local_nav_datas_.end(); local_nav_data_it++) {
        const auto index =
            (*local_nav_data_it)->GetConstNavDataDataRef()->GetIndex();
        if (index != selected_nav_data_index_) {
          ZINFO << "Remove newest unselected nav_data "
                << std::to_string(index);
          lock.Unlock();
          if (!RemoveNavData(index)) {
            ZERROR << "Failed to remove nav_data.";
            return false;
          }
          lock.Lock();
          break;
        }
      }
    }
    local_nav_datas_.emplace_back(new_nav_data);
    std::sort(local_nav_datas_.begin(), local_nav_datas_.end(),
              [](const LocalNavData::SPtr& a, const LocalNavData::SPtr& b) {
                return a->GetConstNavDataDataRef()->GetIndex() <
                       b->GetConstNavDataDataRef()->GetIndex();
              });
  } else {
    std::sort(local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
              [](const uint32_t& a, const uint32_t& b) { return a < b; });
    while (local_nav_data_indexs_.size() >= nav_data_count_limit_) {
      for (auto local_nav_data_it = local_nav_data_indexs_.begin();
           local_nav_data_it != local_nav_data_indexs_.end();
           local_nav_data_it++) {
        const auto index = *local_nav_data_it;
        if (index != selected_nav_data_index_) {
          ZINFO << "Remove newest unselected nav_data "
                << std::to_string(index);
          lock.Unlock();
          if (!RemoveNavData(index)) {
            ZERROR << "Failed to remove nav_data.";
            return false;
          }
          lock.Lock();
          break;
        }
      }
    }
    local_nav_data_indexs_.emplace_back(
        new_nav_data->GetConstNavDataDataRef()->GetIndex());
    std::sort(local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
              [](const uint32_t& a, const uint32_t& b) { return a < b; });
  }

  ZINFO << "Add new nav_data succeed.";
  return true;
}

bool LocalNavDataManager::UpdateNavData(
    const NavData::SPtr& nav_data) {
  if (nav_data == nullptr) {
    ZERROR << "Input data empty.";
    return false;
  }

  auto nav_data_index = nav_data->GetIndex();

  if (kPreloadLocalNavDataIntoMemory_) {
    auto res =
        find_if(local_nav_datas_.begin(), local_nav_datas_.end(),
                [&](const LocalNavData::SPtr& local_nav_data) -> bool {
                  if (local_nav_data == nullptr || !local_nav_data->IsValid()) {
                    return false;
                  }
                  return local_nav_data->GetConstNavDataDataRef()->GetIndex() ==
                         nav_data_index;
                });
    if (res != local_nav_datas_.end()) {
      LocalNavData::SPtr new_nav_data =
          std::make_shared<LocalNavData>(nav_data, localize_path_);
      if (!new_nav_data->IsValid()) {
        ZERROR << "Failed for invalid nav_data.";
        return false;
      }

      if (!new_nav_data->Localize()) {
        ZERROR << "Failed for localization failure.";
        return false;
      }

      (*res) = new_nav_data;
      ZINFO << "Update nav_data " << nav_data_index << " succeed.";
    } else {
      ZERROR << "Nav data for " << nav_data_index << " is not founded.";
      return false;
    }
  } else {
    auto res = find_if(
        local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
        [&](const uint32_t& index) -> bool { return index == nav_data_index; });
    if (res != local_nav_data_indexs_.end()) {
      LocalNavData::SPtr new_nav_data =
          std::make_shared<LocalNavData>(nav_data, localize_path_);
      if (!new_nav_data->IsValid()) {
        ZERROR << "Failed for invalid nav_data.";
        return false;
      }

      if (!new_nav_data->Localize()) {
        ZERROR << "Failed for localization failure.";
        return false;
      }

      ZINFO << "Update nav_data " << nav_data_index << " succeed.";
    } else {
      ZERROR << "Nav data for " << nav_data_index << " is not founded.";
      return false;
    }
  }

  return true;
}

bool LocalNavDataManager::RemoveNavData(const uint32_t& index) {
  LocalNavData::SPtr ptr(nullptr);
  WriteLocker lock(access_);

  auto clear_localization = [&]() -> void {
    const auto file_name = std::to_string(index) + LocalNavData::kSuffix_;
    const auto link_file_name =
        Time::DebugString(static_cast<double>(index)) + LocalNavData::kSuffix_;
    atomic_bool tmp;
    const auto slam_file_name =
        std::to_string(index) + LocalNavData::kSlamFileSuffix_;
    auto files = localize_path_ + file_name + " " + localize_path_ +
                 link_file_name + " " + localize_path_ + slam_file_name;
    if (index == selected_nav_data_index_) {
      files += " " + localize_path_ + file_name + kSelectedNavDataSuffix_;
      selected_nav_data_index_ = 0;
    }
    LocalNavData::ClearLocalization(files, tmp);
  };

  if (kPreloadLocalNavDataIntoMemory_) {
    auto res = find_if(
        local_nav_datas_.begin(), local_nav_datas_.end(),
        [&](const LocalNavData::SPtr& local_nav_data) -> bool {
          if (local_nav_data == nullptr || !local_nav_data->IsValid()) {
            return false;
          }
          return local_nav_data->GetConstNavDataDataRef()->GetIndex() == index;
        });
    if (res != local_nav_datas_.end()) {
      clear_localization();
      local_nav_datas_.erase(res);
    } else {
      ZERROR << "Nav data for " << index << " is not founded.";
      return false;
    }
  } else {
    auto res = find_if(
        local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
        [&](const uint32_t& _index) -> bool { return _index == index; });
    if (res != local_nav_data_indexs_.end()) {
      clear_localization();
      local_nav_data_indexs_.erase(res);
    } else {
      ZERROR << "Nav data for " << index << " is not founded.";
      return false;
    }
  }
  return true;
}

bool LocalNavDataManager::SelectNavData(const uint32_t& index) {
  WriteLocker lock(access_);
  if (index == selected_nav_data_index_) {
    ZINFO << "Nav data " << index << " has already been selected.";
    return true;
  }

  auto select_func = [&]() -> bool {
    const auto file_name = std::to_string(index) + LocalNavData::kSuffix_;
    std::string cmd = "rm -f " + localize_path_ + "*" + kSelectedNavDataSuffix_;
    ZimaRunCommand(cmd);
    cmd = "ln -s " + localize_path_ + file_name + " " + localize_path_ +
          file_name + kSelectedNavDataSuffix_;
    if (ZimaRunCommand(cmd) != 0) {
      ZWARN << "Select nav_data \"" << index << "\" failed.";
      return false;
    }
    selected_nav_data_index_ = index;
    return true;
  };

  if (kPreloadLocalNavDataIntoMemory_) {
    auto res = find_if(
        local_nav_datas_.begin(), local_nav_datas_.end(),
        [&](const LocalNavData::SPtr& local_nav_data) -> bool {
          if (local_nav_data == nullptr || !local_nav_data->IsValid()) {
            return false;
          }
          return local_nav_data->GetConstNavDataDataRef()->GetIndex() == index;
        });
    if (res != local_nav_datas_.end()) {
      if (!select_func()) {
        return false;
      };
    } else {
      ZERROR << "Nav data for " << index << " is not founded.";
      return false;
    }
  } else {
    auto res = find_if(
        local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
        [&](const uint32_t& _index) -> bool { return _index == index; });
    if (res != local_nav_data_indexs_.end()) {
      if (!select_func()) {
        return false;
      };
    } else {
      ZERROR << "Nav data for " << index << " is not founded.";
      return false;
    }
  }
  return true;
}

bool LocalNavDataManager::UnSelectNavData() {
  std::string cmd = "rm -f " + localize_path_ + "*" + kSelectedNavDataSuffix_;
  if (ZimaRunCommand(cmd) != 0) {
    ZWARN << "Un-select nav_data failed.";
    return false;
  }
  ZINFO << "Un-select nav_data succeed.";
  selected_nav_data_index_ = 0;
  return true;
}

LocalNavData::SPtr LocalNavDataManager::GetNavData(
    const uint32_t& index) {
  LocalNavData::SPtr ptr(nullptr);
  // ZINFO << (ptr == nullptr);
  ReadLocker lock(access_);
  if (kPreloadLocalNavDataIntoMemory_) {
    auto res = find_if(
        local_nav_datas_.begin(), local_nav_datas_.end(),
        [&](const LocalNavData::SPtr& local_nav_data) -> bool {
          if (local_nav_data == nullptr || !local_nav_data->IsValid()) {
            return false;
          }
          return local_nav_data->GetConstNavDataDataRef()->GetIndex() == index;
        });
    if (res != local_nav_datas_.end()) {
      ptr = *res;
    } else {
      ZERROR << "Nav data for " << index << " is not founded.";
    }
  } else {
    auto res = find_if(
        local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
        [&](const uint32_t& _index) -> bool { return _index == index; });
    if (res != local_nav_data_indexs_.end()) {
      const auto file_name = std::to_string(index) + LocalNavData::kSuffix_;
      LocalNavData::SPtr nav_data =
          std::make_shared<LocalNavData>(nullptr, localize_path_);
      if (nav_data->LoadFromLocal(file_name) && nav_data->IsValid()) {
        ptr = nav_data;
      } else {
        ZERROR << "Index " << std::to_string(index)
               << " corresponding data invalid, remove it.";
        RemoveNavData(index);
      }
    } else {
      ZERROR << "Nav data for " << index << " is not founded.";
    }
  }

  return ptr;
}

uint32_t LocalNavDataManager::GetSelectedNavDataIndex() const {
  ReadLocker lock(access_);
  return selected_nav_data_index_;
}

std::vector<uint32_t> LocalNavDataManager::GetNavDataIndexList() const {
  ReadLocker lock(access_);
  std::vector<uint32_t> index_list;
  if (kPreloadLocalNavDataIntoMemory_) {
    for (auto&& local_nav_data : local_nav_datas_) {
      index_list.emplace_back(
          local_nav_data->GetConstNavDataDataRef()->GetIndex());
    }
  } else {
    for (auto&& index : local_nav_data_indexs_) {
      index_list.emplace_back(index);
    }
  }
  return index_list;
}

void LocalNavDataManager::LoadNavDatas(const std::vector<std::string>& names) {
  ZGINFO << "Start loading nav_data from " << localize_path_;
  WriteLocker lock(access_);
  local_nav_datas_.clear();
  for (auto&& name : names) {
    LocalNavData::SPtr nav_data =
        std::make_shared<LocalNavData>(nullptr, localize_path_);
    if (nav_data->LoadFromLocal(name) && nav_data->IsValid()) {
      // ZINFO <<
      // std::to_string(nav_data->GetConstNavDataDataRef()->GetIndex());
      // nav_data->GetConstNavDataDataRef()
      //     ->GetOptimizedSlamValueGridMap2DRef()
      //     ->Print(__FILE__, __FUNCTION__, __LINE__);
      // nav_data->GetConstNavDataDataRef()->GetRawSlamValueGridMap2DRef()->Print(
      //     __FILE__, __FUNCTION__, __LINE__);
      if (kPreloadLocalNavDataIntoMemory_) {
        local_nav_datas_.emplace_back(nav_data);
      } else {
        local_nav_data_indexs_.emplace_back(
            nav_data->GetConstNavDataDataRef()->GetIndex());
      }
    }
  }
  if (kPreloadLocalNavDataIntoMemory_) {
    std::sort(local_nav_datas_.begin(), local_nav_datas_.end(),
              [](const LocalNavData::SPtr& a, const LocalNavData::SPtr& b) {
                return a->GetConstNavDataDataRef()->GetIndex() <
                       b->GetConstNavDataDataRef()->GetIndex();
              });
    ZGINFO << "Finish loading, loaded " << local_nav_datas_.size()
           << " nav_datas.";
  } else {
    std::sort(local_nav_data_indexs_.begin(), local_nav_data_indexs_.end(),
              [](const uint32_t& a, const uint32_t& b) { return a < b; });
    ZGINFO << "Finish loading, loaded " << local_nav_data_indexs_.size()
           << " nav_datas.";
  }
}

}  // namespace zima
