/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/system/file.h"

#include <dirent.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <sys/file.h>
#include <sys/stat.h>

#include "zima/hal/system/cmd_line.h"
#include "zima/logger/logger.h"

namespace zima {

bool FileSystemHelper::IsFileOrDirectoryExists(const std::string &file_or_dir) {
  struct stat buf;
  return (stat(file_or_dir.c_str(), &buf) == 0);
}

bool FileSystemHelper::CreateDirectory(const std::string& directory) {
  auto dir = directory;
  if (dir.empty()) {
    ZERROR << "Input directory empty.";
    return false;
  }

  if (dir.back() != '/') {
    dir += "/";
  }

  ZGINFO << "Try to create directory \"" << dir << "\".";
  std::string cmd = "mkdir -p " + dir;
  auto ret = ZimaRunCommand(cmd);
  if (ret != 0) {
    ZERROR << "Create directory \"" << dir << "\" failed.";
  }

  return ret == 0;
}

bool FileSystemHelper::GetNamesInDirectory(const std::string &directory,
                                           std::vector<std::string> &names,
                                           const GetNameFlag &flag) {
  DIR *dir;
  struct dirent *ptr;

  if ((dir = opendir(directory.c_str())) == NULL) {
    ZERROR << "Open directory " << directory << " failed.";
    return false;
  }

  names.clear();
  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) {
      continue;
    } else if (ptr->d_type == 4) {
      // Directory
      if (flag == GetNameFlag::kAllName || flag == GetNameFlag::kDirectoryName) {
        names.push_back(ptr->d_name);
      }
      continue;
    } else if (ptr->d_type == 8) {
      // Normal file
      if (flag == GetNameFlag::kAllName || flag == GetNameFlag::kFileName) {
        names.push_back(ptr->d_name);
      }
      continue;
    } else if (ptr->d_type == 10) {
      // Link file
      if (flag == GetNameFlag::kAllName || flag == GetNameFlag::kLinkFileName) {
        names.push_back(ptr->d_name);
      }
      continue;
    }
  }
  closedir(dir);

  return true;
}

bool FileSystemHelper::GetFilesNamesInDirectory(
    const std::string &directory, std::vector<std::string> &names) {
  return GetNamesInDirectory(directory, names, GetNameFlag::kFileName);
}

bool FileSystemHelper::GetDirectoryNamesInDirectory(const std::string& directory,
                                  std::vector<std::string>& names) {
  return GetNamesInDirectory(directory, names, GetNameFlag::kDirectoryName);
}

bool FileSystemHelper::GetLinkFilesNamesInDirectory(
    const std::string &directory, std::vector<std::string> &names) {
  return GetNamesInDirectory(directory, names, GetNameFlag::kLinkFileName);
}

bool FileSystemHelper::RemoveDirectoryOrFile(
    const std::vector<std::string> &names) {
  bool succeed = true;
  for (auto &&name : names) {
    std::string cmd = "rm -rf " + name;
    auto ret = ZimaRunCommand(cmd);
    if (ret != 0) {
      ZERROR << "Remove \"" << name << "\" failed.";
      succeed = false;
    }
  }
  return succeed;
}

bool FileSystemHelper::RemoveDirectoryOrFile(const std::string &name) {
  std::vector<std::string> names({name});
  return RemoveDirectoryOrFile(names);
}

bool LocalTextFileWrapper::Open() {
  if (file_dir_.empty()) {
    ZERROR << "File \"" << file_full_name_ << "\" does not have a dir prefix.";
    return false;
  }
  if (file_name_.empty()) {
    ZERROR << "File \"" << file_full_name_ << "\" does not have a file name.";
    return false;
  }

  if (!FileSystemHelper::IsFileOrDirectoryExists(file_dir_)) {
    if (!FileSystemHelper::CreateDirectory(file_dir_)) {
      ZERROR << "Can not open " << file_full_name_ << ".";
      return false;
    }
  }

  p_file_ = fopen(file_full_name_.c_str(), mode_.c_str());
  if (p_file_ == nullptr) {
    ZWARN << "Open " << file_full_name_ << " failed.";
  } else {
    opened_ = true;
    return true;
  }
  return false;
}

bool LocalTextFileWrapper::Close() {
  if (opened_) {
    fclose(p_file_);
    opened_ = false;
  }
  return true;
}

bool LocalTextFileReader::ReadWholeFile(std::string &output) {
  if (!opened_) {
    Open();
  }
  if (!opened_) {
    return false;
  }

  if (flock(fileno(p_file_), LOCK_SH) != 0) {
    ZWARN << "Failed to lock " << file_full_name_ << ".";
    return false;
  }

  fseek(p_file_, 0, SEEK_SET);
  bool read_fail = false;
  while (!feof(p_file_)) {
    const size_t buffer_size = 1;
    char buf[1];
    auto _read_len = fread(buf, buffer_size, 1, p_file_);
    if (_read_len > 0) {
      // ZWARN << "Read buffer: " << std::string(1, buf[0]);
      output += std::string(1, buf[0]);
    } else if (_read_len == 0) {
      // ZINFO << "Read finish";
      break;
    } else {
      ZWARN << "Read buffer invalid: " << _read_len;
      read_fail = true;
      break;
    }
  }

  if (flock(fileno(p_file_), LOCK_UN) != 0) {
    ZWARN << "Failed to unlock " << file_full_name_ << ".";
  }

  Close();

  return !read_fail;
}

bool LocalTextFileWritter::WriteWholeFile(const std::string &input) {
  if (!opened_) {
    Open();
  }
  if (!opened_) {
    return false;
  }

  if (flock(fileno(p_file_), LOCK_SH) != 0) {
    ZWARN << "Failed to lock " << file_full_name_ << ".";
    return false;
  }

  fseek(p_file_, 0, SEEK_SET);
  bool write_fail = false;

  auto _write_len = fwrite(input.c_str(), input.size(), 1, p_file_);
  if (_write_len != 1) {
    ZWARN << "Only " << _write_len << " bytes are written.";
    write_fail = true;
  }

  if (flock(fileno(p_file_), LOCK_UN) != 0) {
    ZWARN << "Failed to unlock " << file_full_name_ << ".";
  }

  Close();

  return !write_fail;
}

bool LocalProtoFileReader::GetProtoFromASCIIFile(
    google::protobuf::Message *message) {
  if (!opened_) {
    Open();
  }
  if (!opened_) {
    return false;
  }

  if (flock(fileno(p_file_), LOCK_SH) != 0) {
    ZWARN << "Failed to lock " << file_full_name_ << ".";
    return false;
  }

  fseek(p_file_, 0, SEEK_SET);

  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;

  ZeroCopyInputStream *input = new FileInputStream(fileno(p_file_));
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    ZGWARN << "Failed to parse file " << file_full_name_ << " as text proto.";
  }
  delete input;

  if (flock(fileno(p_file_), LOCK_UN) != 0) {
    ZWARN << "Failed to unlock " << file_full_name_ << ".";
  }

  Close();

  return success;
}

bool LocalProtoFileReader::GetProtoFromBinaryFile(
    google::protobuf::Message *message) {
  if (file_dir_.empty()) {
    ZERROR << "File \"" << file_full_name_ << "\" does not have a dir prefix.";
    return false;
  }
  if (file_name_.empty()) {
    ZERROR << "File \"" << file_full_name_ << "\" does not have a file name.";
    return false;
  }

  if (!FileSystemHelper::IsFileOrDirectoryExists(file_dir_)) {
    if (!FileSystemHelper::CreateDirectory(file_dir_)) {
      ZERROR << "Can not open " << file_full_name_ << ".";
      return false;
    }
  }

  std::fstream input(file_full_name_, std::ios::in | std::ios::binary);
  if (!input.good()) {
    ZERROR << "Failed to open file " << file_full_name_ << " in binary mode.";
    return false;
  }
  auto fd = static_cast<FdHelper &>(*input.rdbuf()).Fd();

  if (flock(fd, LOCK_EX) != 0) {
    ZWARN << "Failed to lock " << file_full_name_ << ".";
    return false;
  }

  auto ret = message->ParseFromIstream(&input);
  if (!ret) {
    ZGWARN << "Failed to parse file " << file_full_name_ << " as binary proto.";
  }

  if (flock(fd, LOCK_UN) != 0) {
    ZWARN << "Failed to unlock " << file_full_name_ << ".";
  }

  return ret;
}

bool LocalProtoFileWriter::SetProtoToASCIIFile(
    const google::protobuf::Message &message) {
  if (!opened_) {
    Open();
  }
  if (!opened_) {
    return false;
  }

  if (flock(fileno(p_file_), LOCK_EX) != 0) {
    ZWARN << "Failed to lock " << file_full_name_ << ".";
    return false;
  }

  using google::protobuf::TextFormat;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyOutputStream;

  ZeroCopyOutputStream *output = new FileOutputStream(fileno(p_file_));
  bool success = TextFormat::Print(message, output);
  if (!success) {
    ZWARN << "Failed to write text proto to file " << file_full_name_;
  }
  delete output;

  if (flock(fileno(p_file_), LOCK_UN) != 0) {
    ZWARN << "Failed to unlock " << file_full_name_ << ".";
  }

  Close();

  return success;
}

bool LocalProtoFileWriter::SetProtoToBinaryFile(
    const google::protobuf::Message &message) {
  if (file_dir_.empty()) {
    ZERROR << "File \"" << file_full_name_ << "\" does not have a dir prefix.";
    return false;
  }
  if (file_name_.empty()) {
    ZERROR << "File \"" << file_full_name_ << "\" does not have a file name.";
    return false;
  }

  if (!FileSystemHelper::IsFileOrDirectoryExists(file_dir_)) {
    if (!FileSystemHelper::CreateDirectory(file_dir_)) {
      ZERROR << "Can not open " << file_full_name_ << ".";
      return false;
    }
  }

  std::fstream output(file_full_name_,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  auto fd = static_cast<FdHelper &>(*output.rdbuf()).Fd();

  if (flock(fd, LOCK_EX) != 0) {
    ZWARN << "Failed to lock " << file_full_name_ << ".";
    return false;
  }

  auto ret = message.SerializeToOstream(&output);

  if (flock(fd, LOCK_UN) != 0) {
    ZWARN << "Failed to unlock " << file_full_name_ << ".";
  }

  return ret;
}

bool LocalJsonFileLoader::GetJsonFromFile(Json &json) {
  std::string json_str;
  if (!ReadWholeFile(json_str)) {
    ZWARN << "Failed to get json string.";
    return false;
  }

  // ZINFO << "Json str: " << json_str;

  if (!JsonHelper::TryParse(json_str, json)) {
    return false;
  }

  return true;
}

bool LocalJsonFileWritter::SetJsonToFile(const Json &json) {
  std::string json_str = json.dump(2);

  if (!WriteWholeFile(json_str)) {
    ZWARN << "Failed to write json string.";
    return false;
  }

  return true;
}

}  // namespace zima
