/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_FILE_H
#define ZIMA_FILE_H

#include <cstdio>
#include <fstream>
#include <string>

#include "google/protobuf/message.h"
#include "zima/common/json.h"

namespace zima {

class FileSystemHelper {
 public:
  FileSystemHelper() = delete;
  ~FileSystemHelper() = delete;

  enum GetNameFlag {
    kFileName,
    kDirectoryName,
    kLinkFileName,
    kAllName,
  };

  static bool IsFileOrDirectoryExists(const std::string& file_or_dir);
  static bool CreateDirectory(const std::string& directory);
  static bool GetNamesInDirectory(const std::string& directory,
                                  std::vector<std::string>& names,
                                  const GetNameFlag& flag);
  static bool GetFilesNamesInDirectory(const std::string& directory,
                                       std::vector<std::string>& names);
  static bool GetDirectoryNamesInDirectory(const std::string& directory,
                                           std::vector<std::string>& names);
  static bool GetLinkFilesNamesInDirectory(const std::string& directory,
                                           std::vector<std::string>& names);
  static bool RemoveDirectoryOrFile(const std::vector<std::string>& names);
  static bool RemoveDirectoryOrFile(const std::string& name);
};

class FdHelper : public std::filebuf {
 public:
  int Fd() { return _M_file.fd(); }
};

class LocalTextFileWrapper {
 public:
  LocalTextFileWrapper() = delete;

  ~LocalTextFileWrapper() { Close(); }

  bool IsOpened() const { return opened_; }

 protected:
  bool Open();

  bool Close();

  LocalTextFileWrapper(const std::string& file_dir,
                       const std::string& file_name,
                       const std::string& mode = "r",
                       const bool& open_on_creation = true)
      : file_dir_(file_dir),
        file_name_(file_name),
        file_full_name_(file_dir + file_name),
        p_file_(nullptr),
        mode_(mode),
        opened_(false) {
    if (open_on_creation) {
      Open();
    }
  }

  std::string file_dir_;
  std::string file_name_;
  std::string file_full_name_;
  FILE* p_file_;
  std::string mode_;
  bool opened_;
};

class LocalTextFileReader : public LocalTextFileWrapper {
 public:
  LocalTextFileReader() = delete;
  LocalTextFileReader(const std::string& file_dir, const std::string& file_name,
                      const bool& open_on_creation = true)
      : LocalTextFileWrapper(file_dir, file_name, "r", open_on_creation){};
  ~LocalTextFileReader() = default;

  bool ReadWholeFile(std::string& output);
};

class LocalTextFileWritter : public LocalTextFileWrapper {
 public:
  LocalTextFileWritter() = delete;
  LocalTextFileWritter(const std::string& file_dir, const std::string& file_name,
                      const bool& open_on_creation = true)
      : LocalTextFileWrapper(file_dir, file_name, "w", open_on_creation){};
  ~LocalTextFileWritter() = default;

  bool WriteWholeFile(const std::string& input);
};

class LocalProtoFileReader : public LocalTextFileWrapper {
 public:
  LocalProtoFileReader() = delete;
  LocalProtoFileReader(const std::string& file_dir,
                       const std::string& file_name,
                       const bool& open_on_creation = true)
      : LocalTextFileWrapper(file_dir, file_name, "r", open_on_creation){};

  bool GetProtoFromASCIIFile(google::protobuf::Message* message);
  bool GetProtoFromBinaryFile(google::protobuf::Message* message);
};

class LocalProtoFileWriter : public LocalTextFileWrapper {
 public:
  LocalProtoFileWriter() = delete;
  LocalProtoFileWriter(const std::string& file_dir,
                       const std::string& file_name,
                       const bool& open_on_creation = true)
      : LocalTextFileWrapper(file_dir, file_name, "w", open_on_creation){};

  bool SetProtoToASCIIFile(const google::protobuf::Message& message);
  bool SetProtoToBinaryFile(const google::protobuf::Message& message);
};

class LocalJsonFileLoader : protected LocalTextFileReader {
 public:
  LocalJsonFileLoader() = delete;
  LocalJsonFileLoader(const std::string& file_dir, const std::string& file_name,
                      const bool& open_on_creation = true)
      : LocalTextFileReader(file_dir, file_name, open_on_creation){};

  bool GetJsonFromFile(Json& json);
};

class LocalJsonFileWritter : protected LocalTextFileWritter {
 public:
  LocalJsonFileWritter() = delete;
  LocalJsonFileWritter(const std::string& file_dir,
                       const std::string& file_name,
                       const bool& open_on_creation = true)
      : LocalTextFileWritter(file_dir, file_name, open_on_creation){};

  bool SetJsonToFile(const Json& json);
};

}  // namespace zima

#endif  // ZIMA_FILE_H
