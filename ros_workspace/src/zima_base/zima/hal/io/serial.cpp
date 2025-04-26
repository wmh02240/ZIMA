/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/io/serial.h"

#include <fcntl.h>
#include <termios.h>

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

Serial::Serial(const std::string& serial_port, const int& baud)
    : access_(std::make_shared<ReadWriteLock>()),
      fd_(-1),
      serial_port_str_(serial_port),
      serial_port_baud_(baud),
      data_access_(std::make_shared<ReadWriteLock>()) {
  FD_ZERO(&fd_set_read_);

  read_timeout_.tv_sec = 1;
  read_timeout_.tv_usec = 0;
}

Serial::~Serial() { Close(); }

bool Serial::IsOpened() {
  ReadLocker lock(access_);
  return fd_ >= 0;
  // return serial_ptr_ != nullptr;
}

bool Serial::Open() {
  if (IsOpened()) {
    ZINFO << "Serial port " << serial_port_str_ << " is already opened.";
    return true;
  }
  WriteLocker lock(access_);

  fd_ = open(serial_port_str_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    ZERROR << "Serial port " << serial_port_str_ << " open failed.";
    return false;
  }

  // Initialize for serial port.
  struct termios newtio, oldtio;
  if (tcgetattr(fd_, &oldtio) != 0) {
    ZERROR << "Serial port " << serial_port_str_ << " initialize failed.";
    ZERROR << "tcgetattr(fd_,&oldtio) -> " << tcgetattr(fd_, &oldtio);
    close(fd_);
    return false;
  }
  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  newtio.c_cflag |= CS8;
  newtio.c_cflag &= ~PARENB;

  cfsetispeed(&newtio, B115200);
  cfsetospeed(&newtio, B115200);

  newtio.c_cflag &= ~CSTOPB;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd_, TCIFLUSH);

  if ((tcsetattr(fd_, TCSANOW, &newtio)) != 0) {
    ZERROR << "Serial port " << serial_port_str_ << " initialize failed.";
    ZERROR << "com set error";
    close(fd_);
    return false;
  }

  FD_ZERO(&fd_set_read_);
  FD_SET(fd_, &fd_set_read_);

  ZINFO << "Serial port " << serial_port_str_ << " opened.";
  return true;
}

bool Serial::Close() {
  if (!IsOpened()) {
    ZINFO << "Serial port " << serial_port_str_ << " is already closed.";
    return true;
  }
  WriteLocker lock(access_);
  close(fd_);
  return true;
}

bool Serial::Read() {
  if (!IsOpened()) {
    ZWARN << "Serial port " << serial_port_str_ << " is not opened yet.";
    return false;
  }

  ReadLocker lock(access_);

  read_timeout_.tv_sec = 1;
  read_timeout_.tv_usec = 0;
  auto fs_sel = select(fd_ + 1, &fd_set_read_, NULL, NULL, &read_timeout_);
  // ZINFO << "Select " << fd_ << " return " << fs_sel;
  if (fs_sel) {
    uint8_t recv_buf = 0;
    auto len = read(fd_, &recv_buf, 1);
    lock.Unlock();
    if (len > 0) {
      // ZINFO << "Read " << len << " bytes.";
      // ZINFO << "Read for: " << UintToHexString(recv_buf);
      ParseFrame(recv_buf);
      return true;
    }
  }
  return false;
}

bool Serial::Write(const BytesFrame& write_bytes) {
  if (!IsOpened()) {
    ZWARN << "Serial port " << serial_port_str_ << " is not opened yet.";
    return false;
  }

  constexpr uint32_t kBufMaxSize = 128;
  if (write_bytes.size() > kBufMaxSize) {
    ZERROR << "Bytes more than buf size.";
    return false;
  }

  ReadLocker lock(access_);

  uint8_t write_buf[kBufMaxSize];

  uint32_t index = 0;
  // std::string debug_str = "Write ";
  for (auto && byte : write_bytes) {
    write_buf[index++] = byte;
    // debug_str += UintToHexString(write_buf[index - 1]) + " ";
  }
  // ZINFO << debug_str;

  auto ret = write(fd_, write_buf, index);
  if (ret != index) {
    ZERROR << "Plan to write " << index << " bytes but only " << ret
           << " bytes were written.";
    return false;
  }
  // ZINFO << "Write " << index << " bytes success.";

  return true;
}


bool Serial::ParseFrame(const uint8_t& recv_byte) {
  WriteLocker data_lock(data_access_);
  ZINFO;
  cache_bytes_.clear();
  valid_frames_data_.clear();
  return true;
}

bool Serial::GetReadValidFramesData(BytesFrames& frames_data) {
  WriteLocker data_lock(data_access_);
  if (!valid_frames_data_.empty()) {
    frames_data.clear();
    frames_data.swap(valid_frames_data_);
    return true;
  }
  return false;
}

bool Serial::CacheWriteFrames(const BytesFrame& frames) {
  WriteLocker data_lock(data_access_);
  write_frames_.emplace_back(frames);
  return true;
}

bool Serial::GetWriteFrames(BytesFrames& frames) {
  WriteLocker data_lock(data_access_);
  if (!write_frames_.empty()) {
    frames.clear();
    frames.swap(write_frames_);
    return true;
  }
  return false;
}

std::string Serial::DebugString(const BytesFrame& frame) {
  std::string str;
  for (auto&& byte : frame) {
    str += UintToHexString(byte) + " ";
  }

  return str;
}

}  // namespace zima
