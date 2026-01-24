/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CLEANING_RECORD_H
#define ZIMA_CLEANING_RECORD_H

#include <atomic>

#include "zima/common/macro.h"
#include "zima/robot_data/operation_data.h"

namespace zima {

class CleaningRecord {
 public:
  CleaningRecord() = delete;
  CleaningRecord(const OperationData::SPtr& operation_data,
                 const std::string& localize_path,
                 const bool& localized = false);
  ~CleaningRecord();

  using SPtr = std::shared_ptr<CleaningRecord>;

  bool IsValid() const;
  bool Localize();
  static void ClearLocalization(const std::string& file_names,
                                atomic_bool& localized_flag);

  bool LoadFromLocal(const std::string& name);

  OperationData::SCPtr GetConstRecordDataRef();

  static const std::string kSuffix_;

 private:
  ReadWriteLock::SPtr access_;
  OperationData::SPtr record_data_;
  atomic_bool valid_;
  atomic_bool localized_;
  const std::string kLocalizePath_;
};

class CleaningRecordManager {
  DECLARE_SINGLETON(CleaningRecordManager)
 public:
  ~CleaningRecordManager();

  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kCleaningRecordPathKey_;
    std::string cleaning_record_path_;
    static const std::string kCleaningRecordCountLimitKey_;
    uint cleaning_record_count_limit_;
  };

  using CleaningRecords = std::deque<CleaningRecord::SPtr>;

  class RecordSummary {
   public:
    RecordSummary() = delete;
    RecordSummary(const uint32_t& start_timestamp,
                  const OperationData::OperationType& type,
                  const OperationData::OperationResult& result,
                  const float& clean_area_size, const uint32_t& duration)
        : start_timestamp_(start_timestamp),
          type_(type),
          result_(result),
          clean_area_size_(clean_area_size),
          duration_(duration) {}
    ~RecordSummary() = default;

    using SPtr = std::shared_ptr<RecordSummary>;

    std::string DebugString();

    uint32_t start_timestamp_;
    OperationData::OperationType type_;
    OperationData::OperationResult result_;
    float clean_area_size_;
    uint32_t duration_;
  };

  using RecordSummaries = std::deque<RecordSummary::SPtr>;

  RecordSummaries GetRecordsSummaryList() const;

  bool LocalizeRecordsSummaryList();
  bool LoadRecordsSummaryList();
  void ClearRecordsSummaryListLocalization();
  void RecheckAndTrimRecordsSummaryListFromRecordsData(
      const uint16_t& limit_size);

  bool AddNewRecord(const OperationData::SPtr& operation_data);
  RecordSummary::SPtr GetRecordSummary(const uint32_t& start_timestamp);
  CleaningRecord::SPtr GetRecordData(const uint32_t& start_timestamp);
  bool RemoveRecords(const std::vector<uint32_t>& start_timestamps);
  bool TrimRecords(const uint16_t& limit_size);

  static uint32_t ToUIntTimeStamp(const double& timestamp) {
    return static_cast<uint32_t>(floor(timestamp));
  };

  static const std::string kRecordListFileName_;

 private:
  ReadWriteLock::SPtr access_;
  std::string localize_path_;
  uint16_t record_count_limit_;
  RecordSummaries record_summaries_;
};

class RecordSummarySerializer {
 public:
  RecordSummarySerializer() = delete;

  static ZimaProto::OperationData::PCleaningRecordSummary ToProto(
      const CleaningRecordManager::RecordSummary::SPtr& summary);
  static bool FromProto(
      CleaningRecordManager::RecordSummary::SPtr& summary,
      const ZimaProto::OperationData::PCleaningRecordSummary& proto);

  static ZimaProto::OperationData::PCleaningRecordSummaryList ToProto(
      const CleaningRecordManager::RecordSummaries& summaries);
  static bool FromProto(
      CleaningRecordManager::RecordSummaries& summaries,
      const ZimaProto::OperationData::PCleaningRecordSummaryList& proto);
};

class RecordSummaryListLoader : public LocalProtoFileReader {
 public:
  RecordSummaryListLoader() = default;
  explicit RecordSummaryListLoader(const std::string& file_dir,
                                   const std::string& file_name)
      : LocalProtoFileReader(file_dir, file_name){};
  ~RecordSummaryListLoader() = default;

  bool LoadData(CleaningRecordManager::RecordSummaries& data);
};

class RecordSummaryListWriter : public LocalProtoFileWriter {
 public:
  RecordSummaryListWriter() = default;
  explicit RecordSummaryListWriter(const std::string& file_dir,
                                   const std::string& file_name)
      : LocalProtoFileWriter(file_dir, file_name){};
  ~RecordSummaryListWriter() = default;

  bool WriteData(const CleaningRecordManager::RecordSummaries& data,
                 const bool& is_binary);
};

}  // namespace zima

#endif  // ZIMA_CLEANING_RECORD_H
