/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot_data/cleaning_record.h"

#include <algorithm>

#include "zima/common/config.h"
#include "zima/hal/system/cmd_line.h"

namespace zima {

const std::string CleaningRecord::kSuffix_ = ".clean_record.pbstream";
const std::string CleaningRecordManager::kRecordListFileName_ =
    "record_list.pb.txt";

CleaningRecord::CleaningRecord(const OperationData::SPtr& operation_data,
                               const std::string& localize_path,
                               const bool& localized)
    : access_(std::make_shared<ReadWriteLock>()),
      localized_(localized),
      kLocalizePath_(localize_path) {
  if (operation_data == nullptr) {
    record_data_ = nullptr;
    valid_.store(false);
    // ZINFO << "Create empty record(localized " << localized_.load() << ").";
  } else {
    record_data_ = std::make_shared<OperationData>(*operation_data);
    record_data_->GetNavMapRef()->RemoveCopySuffix();
    record_data_->GetRawSlamValueGridMap2DRef()->RemoveCopySuffix();
    record_data_->GetOptimizedSlamValueGridMap2DRef()->RemoveCopySuffix();
    valid_.store(true);
    ZGINFO << "Create record(localized " << localized_.load() << "): \n"
           << record_data_->DebugOperationInfo();
  }
}

CleaningRecord::~CleaningRecord() {
  if (valid_.load()) {
    ZGINFO << "Release record(localized " << localized_.load() << "): \n"
           << record_data_->DebugOperationInfo();
  }
}

bool CleaningRecord::IsValid() const { return valid_.load(); }

bool CleaningRecord::Localize() {
  if (!valid_.load()) {
    ZWARN << "Record was not valid.";
    return false;
  }

  if (localized_.load()) {
    ZWARN << "Record " << Time::DebugString(record_data_->GetStartTime())
          << " has been localized.";
    return true;
  }

  ReadLocker lock(access_);

  const auto time = static_cast<time_t>(floor(record_data_->GetStartTime()));
  const auto file_name = std::to_string(time) + kSuffix_;
  const auto link_file_name =
      Time::DebugString(record_data_->GetStartTime()) + kSuffix_;

  OperationDataWriter writter(kLocalizePath_, file_name);
  if (!writter.WriteData(record_data_, true)) {
    ClearLocalization(
        kLocalizePath_ + file_name + " " + kLocalizePath_ + link_file_name,
        localized_);
    localized_.store(false);
    ZWARN << "Record " << Time::DebugString(record_data_->GetStartTime())
          << " localization failed.";
    return false;
  }

  std::string cmd = "ln -s " + kLocalizePath_ + file_name + " " +
                    kLocalizePath_ + link_file_name;
  if (ZimaRunCommand(cmd) != 0) {
    ClearLocalization(
        kLocalizePath_ + file_name + " " + kLocalizePath_ + link_file_name,
        localized_);
    localized_.store(false);
    ZWARN << "Record " << Time::DebugString(record_data_->GetStartTime())
          << " localization failed.";
    return false;
  }

  localized_.store(true);

  return true;
}

void CleaningRecord::ClearLocalization(const std::string& file_names,
                                       atomic_bool& localized_flag) {
  std::string cmd = "rm -rf " + file_names;
  if (ZimaRunCommand(cmd) != 0) {
    ZWARN << "Clear for file \"" << file_names << "\" failed.";
    return;
  }

  localized_flag.store(false);
  ZGINFO << "Clear for file \"" << file_names << "\" succeed.";
  return;
}

bool CleaningRecord::LoadFromLocal(const std::string& name) {
  if (valid_.load()) {
    ZWARN << "This record is already loaded.";
    return false;
  }

  WriteLocker lock(access_);
  OperationDataLoader loader(kLocalizePath_, name);
  auto record_data = std::make_shared<OperationData>(true);
  if (!loader.LoadData(record_data, true)) {
    ZWARN << "Load for record file " << kLocalizePath_ << name << " failed.";
    return false;
  }
  record_data_ = record_data;
  valid_.store(true);
  localized_.store(true);
  // ZINFO << "Load record(localized " << localized_.load() << ")";

  return true;
}

OperationData::SCPtr CleaningRecord::GetConstRecordDataRef() {
  ReadLocker lock(access_);
  return record_data_;
}

std::string CleaningRecordManager::RecordSummary::DebugString() {
  std::string str = "\nStart time: " +
                    Time::DebugString(static_cast<double>(start_timestamp_)) +
                    "(" + std::to_string(start_timestamp_) + ")";
  str += "\nType: ";
  switch (type_) {
    case OperationData::OperationType::kSelectRoomCleaning: {
      str += "Select room cleaning";
      break;
    }
    case OperationData::OperationType::kSelectAreaCleaning: {
      str += "Select area cleaning";
      break;
    }
    case OperationData::OperationType::kAllHouseCleaning: {
      str += "All house cleaning";
      break;
    }
    case OperationData::OperationType::kAllHouseScanning: {
      str += "All house scanning";
      break;
    }
    default: {
      str += "Unknown operation " + std::to_string(type_);
      break;
    }
  }
  str += "\nResult: ";
  switch (result_) {
    case OperationData::OperationResult::kFinishAutoCleaning: {
      str += "Finish";
      break;
    }
    case OperationData::OperationResult::kFinishAutoScanHouse: {
      str += "Finish";
      break;
    }
    case OperationData::OperationResult::kStopped:
    default: {
      str += "Stopped";
      break;
    }
  }
  str += "\nCleaning area: " + FloatToString(clean_area_size_, 2) +
         "m2\nCleaning seconds:" + std::to_string(duration_) + "s (" +
         FloatToString(duration_ / 60, 2) + "min)";

  return str;
}

CleaningRecordManager::Config::Config() : Config(nullptr){};

CleaningRecordManager::Config::Config(const JsonSPtr& json) : config_valid_(false) {
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

bool CleaningRecordManager::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetString(*json, kCleaningRecordPathKey_,
                             cleaning_record_path_)) {
    ZERROR << "Config " << kCleaningRecordPathKey_ << " not found.";
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kCleaningRecordCountLimitKey_,
                           cleaning_record_count_limit_)) {
    ZERROR << "Config " << kCleaningRecordCountLimitKey_ << " not found.";
    return false;
  }

  return true;
}

CleaningRecordManager::CleaningRecordManager()
    : access_(std::make_shared<ReadWriteLock>()) {
  Config config;
  localize_path_ = config.config_valid_ ? config.cleaning_record_path_
                                        : "/tmp/zima_clean_record/";
  record_count_limit_ =
      config.config_valid_ ? config.cleaning_record_count_limit_ : 20;

  if (!FileSystemHelper::IsFileOrDirectoryExists(localize_path_)) {
    FileSystemHelper::CreateDirectory(localize_path_);
  }
  LoadRecordsSummaryList();

  // Re check records.
  if (false) {
    RecheckAndTrimRecordsSummaryListFromRecordsData(record_count_limit_);
  }
}

CleaningRecordManager::RecordSummaries
CleaningRecordManager::GetRecordsSummaryList() const {
  ReadLocker lock(access_);
  return record_summaries_;
}

bool CleaningRecordManager::LocalizeRecordsSummaryList() {
  ZGINFO << "Start updating summary list file \"" << localize_path_
        << kRecordListFileName_;
  ReadLocker lock(access_);
  RecordSummaryListWriter writter(localize_path_, kRecordListFileName_);
  if (!writter.WriteData(record_summaries_, false)) {
    lock.Unlock();
    ClearRecordsSummaryListLocalization();
    ZWARN << "Record summary list file \"" << localize_path_
          << kRecordListFileName_ << "\" localization failed.";
    return false;
  }

  ZGINFO << "Update summary list file \"" << localize_path_
         << kRecordListFileName_ << "\" succeed.";
  return true;
}

bool CleaningRecordManager::LoadRecordsSummaryList() {
  ZGINFO << "Start loading record summary list.";
  RecordSummaryListLoader loader(localize_path_, kRecordListFileName_);
  CleaningRecordManager::RecordSummaries summeries;
  if (!loader.LoadData(summeries)) {
    ZWARN << "Load for summary list file \"" << localize_path_
          << kRecordListFileName_ << "\" failed.";
    return false;
  }

  WriteLocker lock(access_);
  record_summaries_ = summeries;
  std::sort(record_summaries_.begin(), record_summaries_.end(),
            [](const RecordSummary::SPtr& a, const RecordSummary::SPtr& b) {
              return a->start_timestamp_ > b->start_timestamp_;
            });
  ZGINFO << "Finish loading, loaded " << record_summaries_.size()
         << " summaries.";
  return true;
}

void CleaningRecordManager::ClearRecordsSummaryListLocalization() {
  WriteLocker lock(access_);
  std::string cmd = "rm -f " + localize_path_ + kRecordListFileName_;
  if (ZimaRunCommand(cmd) != 0) {
    ZWARN << "Clear for Record summary list \""
          << localize_path_ + kRecordListFileName_ << "\" failed.";
    return;
  }

  ZGINFO << "Clear for Record summary list \""
        << localize_path_ + kRecordListFileName_ << "\" succeed.";
}

void CleaningRecordManager::RecheckAndTrimRecordsSummaryListFromRecordsData(
    const uint16_t& limit_size) {
  ZGINFO << "Start re-check records.";
  std::vector<std::string> file_names;
  if (!FileSystemHelper::GetFilesNamesInDirectory(localize_path_, file_names)) {
    ZERROR << "Directory unavailable.";
    return;
  }

  std::vector<std::string> record_names;
  for (auto&& file_name : file_names) {
    if (StringEndsWith(file_name, CleaningRecord::kSuffix_)) {
      record_names.emplace_back(file_name);
    }
  }

  std::sort(record_names.begin(), record_names.end(),
            [](const std::string& a, const std::string& b) { return a > b; });

  auto get_timestamp = [&](const std::string& name) -> uint32_t {
    auto time_str = name;
    for (auto len = 0u; len < CleaningRecord::kSuffix_.size(); len++) {
      time_str.pop_back();
    }
    return static_cast<uint32_t>(atoi(time_str.c_str()));
  };
  std::vector<uint32_t> timestamps_to_delete;

  RecordSummaries new_summaries;
  uint16_t record_count = 0;
  for (auto&& name : record_names) {
    if (record_count < limit_size) {
      CleaningRecord::SPtr record =
          std::make_shared<CleaningRecord>(nullptr, localize_path_);
      if (record->LoadFromLocal(name) && record->IsValid()) {
        new_summaries.emplace_back(
            std::make_shared<CleaningRecordManager::RecordSummary>(
                ToUIntTimeStamp(
                    record->GetConstRecordDataRef()->GetStartTime()),
                record->GetConstRecordDataRef()->GetOperationType(),
                record->GetConstRecordDataRef()->GetOperationResult(),
                record->GetConstRecordDataRef()
                    ->GetNavMapConstRef()
                    ->GetStepAreaSize(),
                ToUIntTimeStamp(
                    record->GetConstRecordDataRef()->GetDuration())));
        ZGINFO << "Check valid summary: "
               << new_summaries.back()->DebugString();
        record_count++;
      } else {
        ZERROR << "Found invalid record file: " << localize_path_ << name;
        timestamps_to_delete.emplace_back(get_timestamp(name));
      }
    } else {
      ZGINFO << "Remove extra record file: " << localize_path_ << name;
      timestamps_to_delete.emplace_back(get_timestamp(name));
    }
  }
  WriteLocker lock(access_);
  record_summaries_.clear();
  lock.Unlock();
  RemoveRecords(timestamps_to_delete);
  lock.Lock();
  record_summaries_ = new_summaries;
  std::sort(record_summaries_.begin(), record_summaries_.end(),
            [](const RecordSummary::SPtr& a, const RecordSummary::SPtr& b) {
              return a->start_timestamp_ > b->start_timestamp_;
            });
  ZGINFO << "Finish re-checking, " << record_summaries_.size()
         << " valid records.";
  lock.Unlock();
  LocalizeRecordsSummaryList();
}

bool CleaningRecordManager::AddNewRecord(
    const OperationData::SPtr& operation_data) {
  ZGINFO << "Add new record.";
  CleaningRecord::SPtr new_record =
      std::make_shared<CleaningRecord>(operation_data, localize_path_);
  if (!new_record->IsValid()) {
    ZERROR << "Failed for empty record.";
    return false;
  }

  if (!new_record->Localize()) {
    ZERROR << "Failed for localization failure.";
    return false;
  }
  WriteLocker lock(access_);
  record_summaries_.emplace_back(
      std::make_shared<CleaningRecordManager::RecordSummary>(
          ToUIntTimeStamp(new_record->GetConstRecordDataRef()->GetStartTime()),
          new_record->GetConstRecordDataRef()->GetOperationType(),
          new_record->GetConstRecordDataRef()->GetOperationResult(),
          new_record->GetConstRecordDataRef()
              ->GetNavMapConstRef()
              ->GetStepAreaSize(),
          ToUIntTimeStamp(new_record->GetConstRecordDataRef()->GetDuration())));
  ZGINFO << "Add new summary: " << record_summaries_.back()->DebugString();
  std::sort(record_summaries_.begin(), record_summaries_.end(),
            [](const RecordSummary::SPtr& a, const RecordSummary::SPtr& b) {
              return a->start_timestamp_ > b->start_timestamp_;
            });
  ZINFO << "Add new record " << new_record->GetConstRecordDataRef()->GetIndex()
        << " succeed, total " << record_summaries_.size() << " records.";
  if (record_summaries_.size() > record_count_limit_) {
    lock.Unlock();
    TrimRecords(record_count_limit_);
  } else {
    lock.Unlock();
    LocalizeRecordsSummaryList();
  }

  return true;
}

CleaningRecordManager::RecordSummary::SPtr
CleaningRecordManager::GetRecordSummary(const uint32_t& start_timestamp) {
  ReadLocker lock(access_);
  RecordSummary::SPtr ptr(nullptr);
  auto res = find_if(record_summaries_.begin(), record_summaries_.end(),
                     [&](const RecordSummary::SPtr& summary) -> bool {
                       if (summary == nullptr) {
                         return false;
                       }
                       return summary->start_timestamp_ == start_timestamp;
                     });
  if (res != record_summaries_.end()) {
    ptr = *res;
  } else {
    ZERROR << "Summary for " << start_timestamp << " is not founded.";
  }
  return ptr;
}

CleaningRecord::SPtr CleaningRecordManager::GetRecordData(
    const uint32_t& start_timestamp) {
  CleaningRecord::SPtr ptr(nullptr);
  ZGINFO << (ptr == nullptr);
  ReadLocker lock(access_);

  // Load from file.
  CleaningRecord::SPtr record =
      std::make_shared<CleaningRecord>(nullptr, localize_path_);

  const auto file_name =
      std::to_string(start_timestamp) + CleaningRecord::kSuffix_;
  if (record->LoadFromLocal(file_name) && record->IsValid()) {
    ptr = record;
  } else {
    ZERROR << "Record for " << start_timestamp << " is not founded.";
  }

  return ptr;
}

bool CleaningRecordManager::RemoveRecords(
    const std::vector<uint32_t>& start_timestamps) {
  if (start_timestamps.empty()) {
    ZWARN << "Input empty.";
    return false;
  }
  WriteLocker lock(access_);
  for (auto&& start_timestamp : start_timestamps) {
    ZGINFO << "Remove record "
           << Time::DebugString(static_cast<double>(start_timestamp)) << "("
           << start_timestamp << ").";
    {
      const auto file_name =
          std::to_string(start_timestamp) + CleaningRecord::kSuffix_;
      const auto link_file_name =
          Time::DebugString(static_cast<double>(start_timestamp)) +
          CleaningRecord::kSuffix_;
      atomic_bool tmp;
      CleaningRecord::ClearLocalization(
          localize_path_ + file_name + " " + localize_path_ + link_file_name,
          tmp);
    }
    {
      auto res = find_if(record_summaries_.begin(), record_summaries_.end(),
                         [&](const RecordSummary::SPtr& summary) -> bool {
                           if (summary == nullptr) {
                             return false;
                           }
                           return summary->start_timestamp_ == start_timestamp;
                         });
      if (res != record_summaries_.end()) {
        record_summaries_.erase(res);
        ZGINFO << "Remove record "
               << Time::DebugString(static_cast<double>(start_timestamp)) << "("
               << start_timestamp << ") succeed, total "
               << record_summaries_.size() << " records remain.";
      }
    }
  }

  std::sort(record_summaries_.begin(), record_summaries_.end(),
            [](const RecordSummary::SPtr& a, const RecordSummary::SPtr& b) {
              return a->start_timestamp_ > b->start_timestamp_;
            });
  lock.Unlock();
  LocalizeRecordsSummaryList();
  lock.Lock();

  return true;
}

bool CleaningRecordManager::TrimRecords(const uint16_t& limit_size) {
  ZGINFO << "Start trim records.";
  ReadLocker read_lock(access_);
  std::sort(record_summaries_.begin(), record_summaries_.end(),
            [](const RecordSummary::SPtr& a, const RecordSummary::SPtr& b) {
              return a->start_timestamp_ > b->start_timestamp_;
            });

  std::vector<uint32_t> timestamps_to_delete;

  RecordSummaries new_summaries;
  uint16_t record_count = 0;
  for (auto&& summary : record_summaries_) {
    if (record_count < limit_size) {
      new_summaries.emplace_back(summary);
    } else {
      ZGINFO << "Remove extra record: " << summary->start_timestamp_;
      timestamps_to_delete.emplace_back(summary->start_timestamp_);
    }
    record_count++;
  }
  read_lock.Unlock();
  WriteLocker write_lock(access_);
  record_summaries_.clear();
  write_lock.Unlock();
  RemoveRecords(timestamps_to_delete);
  // Now summaries is still empty.
  write_lock.Lock();
  record_summaries_ = new_summaries;
  std::sort(record_summaries_.begin(), record_summaries_.end(),
            [](const RecordSummary::SPtr& a, const RecordSummary::SPtr& b) {
              return a->start_timestamp_ > b->start_timestamp_;
            });
  ZGINFO << "Finish trimming records, " << record_summaries_.size()
         << " valid records.";
  write_lock.Unlock();
  LocalizeRecordsSummaryList();

  return true;
}

ZimaProto::OperationData::PCleaningRecordSummary
RecordSummarySerializer::ToProto(
    const CleaningRecordManager::RecordSummary::SPtr& summary) {
  ZimaProto::OperationData::PCleaningRecordSummary proto;
  if (summary == nullptr) {
    ZERROR << "Data empty.";
    return proto;
  }
  proto.set_start_time(summary->start_timestamp_);
  proto.set_duration(summary->duration_);
  proto.set_area_size(summary->clean_area_size_);
  switch (summary->type_) {
    case OperationData::OperationType::kSelectRoomCleaning: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kSelectRoomCleaning);
      break;
    }
    case OperationData::OperationType::kSelectAreaCleaning: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kSelectAreaCleaning);
      break;
    }
    case OperationData::OperationType::kAllHouseScanning: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kAllHouseScanning);
      break;
    }
    case OperationData::OperationType::kAllHouseCleaning:
    default: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kAllHouseCleaning);
      break;
    }
  }
  switch (summary->result_) {
    case OperationData::OperationResult::kFinishAutoCleaning: {
      proto.set_operation_result(
          ZimaProto::OperationData::POperationResult::kFinishAutoCleaning);
      break;
    }
    case OperationData::OperationResult::kStopped:
    default: {
      proto.set_operation_result(
          ZimaProto::OperationData::POperationResult::kStopped);
      break;
    }
  }

  // ZWARN << summary->DebugString();
  // ZWARN << "\n" << proto.DebugString();
  return proto;
}

bool RecordSummarySerializer::FromProto(
    CleaningRecordManager::RecordSummary::SPtr& summary,
    const ZimaProto::OperationData::PCleaningRecordSummary& proto) {
  if (summary == nullptr) {
    ZERROR << "Summary empty.";
    return false;
  }

  summary->start_timestamp_ = proto.start_time();
  summary->duration_ = proto.duration();
  switch (proto.operation_type()) {
    case ZimaProto::OperationData::POperationType::kSelectRoomCleaning: {
      summary->type_ = OperationData::OperationType::kSelectRoomCleaning;
      break;
    }
    case ZimaProto::OperationData::POperationType::kSelectAreaCleaning: {
      summary->type_ = OperationData::OperationType::kSelectAreaCleaning;
      break;
    }
    case ZimaProto::OperationData::POperationType::kAllHouseScanning: {
      summary->type_ = OperationData::OperationType::kAllHouseScanning;
      break;
    }
    case ZimaProto::OperationData::POperationType::kAllHouseCleaning:
    default: {
      summary->type_ = OperationData::OperationType::kAllHouseCleaning;
      break;
    }
  }

  switch (proto.operation_result()) {
    case ZimaProto::OperationData::POperationResult::kFinishAutoCleaning: {
      summary->result_ = OperationData::OperationResult::kFinishAutoCleaning;
      break;
    }
    case ZimaProto::OperationData::POperationResult::kStopped:
    default: {
      summary->result_ = OperationData::OperationResult::kStopped;
      break;
    }
  }
  return true;
}

ZimaProto::OperationData::PCleaningRecordSummaryList
RecordSummarySerializer::ToProto(
    const CleaningRecordManager::RecordSummaries& summaries) {
  ZimaProto::OperationData::PCleaningRecordSummaryList proto;
  for (auto&& summary : summaries) {
    *proto.add_record_summary_list() = ToProto(summary);
  }
  // ZINFO << "\n" << proto.DebugString();
  return proto;
}

bool RecordSummarySerializer::FromProto(
    CleaningRecordManager::RecordSummaries& summaries,
    const ZimaProto::OperationData::PCleaningRecordSummaryList& proto) {
  summaries.clear();
  for (auto&& summary_proto : proto.record_summary_list()) {
    auto summary_ptr = std::make_shared<CleaningRecordManager::RecordSummary>(
        0, OperationData::OperationType::kAllHouseCleaning,
        OperationData::OperationResult::kStopped, 0, 0);
    if (FromProto(summary_ptr, summary_proto)) {
      summaries.emplace_back(summary_ptr);
    }
  }
  return true;
}

bool RecordSummaryListLoader::LoadData(
    CleaningRecordManager::RecordSummaries& data) {
  ZimaProto::OperationData::PCleaningRecordSummaryList proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (RecordSummarySerializer::FromProto(data, proto)) {
      ZGINFO << "Load " << file_name_ << " success.";
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool RecordSummaryListWriter::WriteData(
    const CleaningRecordManager::RecordSummaries& data, const bool& is_binary) {
  auto proto = RecordSummarySerializer::ToProto(data);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZGINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

}  // namespace zima
