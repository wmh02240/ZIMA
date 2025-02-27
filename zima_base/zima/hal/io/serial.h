/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */
#ifndef ZIMA_SERIAL_H
#define ZIMA_SERIAL_H

#include <deque>
#include <vector>

#include "zima/common/lock.h"

namespace zima {

class Serial {
 public:
  Serial() = delete;
  Serial(const std::string& serial_port, const int& baud);
  ~Serial();

  using BytesFrame = std::vector<uint8_t>;
  using BytesFrames = std::deque<BytesFrame>;

  bool IsOpened();
  virtual bool Open();
  bool Close();
  bool Read();
  bool Write(const BytesFrame& write_bytes);

  virtual bool ParseFrame(const uint8_t& recv_byte);
  bool GetReadValidFramesData(BytesFrames& frames);
  virtual bool CacheWriteFrames(const BytesFrame& frames);
  bool GetWriteFrames(BytesFrames& frames);

  static std::string DebugString(const BytesFrame& frame);

 private:
  ReadWriteLock::SPtr access_;

  int fd_;
  fd_set fd_set_read_;
  std::string serial_port_str_;
  int serial_port_baud_;
  struct timeval read_timeout_;

 protected:
  ReadWriteLock::SPtr data_access_;
  BytesFrame cache_bytes_;
  BytesFrames valid_frames_data_;
  BytesFrames write_frames_;
};

}  // namespace zima

#endif  // ZIMA_SERIAL_H
