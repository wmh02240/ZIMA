/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/lock.h"

#include <sys/syscall.h>  // For thread id.
#include <unistd.h>       // For sysconf.

#include "zima/logger/logger.h"

#define SUPPORT_DEADLOCK_CHECKING

namespace zima {

using namespace std;

int GetThisThreadId() { return syscall(SYS_gettid); }

ReadWriteLock::ReadWriteLock() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  thread_lock_info_.clear();
#else
  thread_lock_info_[0];
#endif  // SUPPORT_DEADLOCK_CHECKING
}

ReadWriteLock::~ReadWriteLock() {}

bool ReadWriteLock::InRead() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  bool in_read = false;
  thread_lock_info_mutex_.lock();
  for (auto &&lock_info : thread_lock_info_) {
    if (lock_info.second.read_count_.load() > 0) {
      in_read = true;
      break;
    }
  }
  thread_lock_info_mutex_.unlock();
  return in_read;
#else
  return thread_lock_info_[0].read_count_.load() > 0;
#endif  // SUPPORT_DEADLOCK_CHECKING
}

bool ReadWriteLock::InReadByThisThread() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  thread_lock_info_mutex_.lock();
  bool in_read = thread_lock_info_[GetThisThreadId()].read_count_.load() > 0;
  thread_lock_info_mutex_.unlock();
  return in_read;
#else
  return thread_lock_info_[0].read_count_.load() > 0;
#endif  // SUPPORT_DEADLOCK_CHECKING
}

uint16_t ReadWriteLock::ReaderCount() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  uint16_t reader_count = 0;
  thread_lock_info_mutex_.lock();
  for (auto &&lock_info : thread_lock_info_) {
    reader_count += lock_info.second.read_count_.load();
  }
  thread_lock_info_mutex_.unlock();
  return reader_count;
#else
  return thread_lock_info_[0].read_count_.load();
#endif  // SUPPORT_DEADLOCK_CHECKING
}

uint16_t ReadWriteLock::ReaderCountForThisThread() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  thread_lock_info_mutex_.lock();
  uint16_t reader_count =
      thread_lock_info_[GetThisThreadId()].read_count_.load();
  thread_lock_info_mutex_.unlock();
  return reader_count;
#else
  return thread_lock_info_[0].read_count_.load();
#endif  // SUPPORT_DEADLOCK_CHECKING
}

bool ReadWriteLock::InWrite() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  bool in_write = false;
  thread_lock_info_mutex_.lock();
  for (auto &&lock_info : thread_lock_info_) {
    if (lock_info.second.in_write_) {
      in_write = true;
      break;
    }
  }
  thread_lock_info_mutex_.unlock();
  return in_write;
#else
  return thread_lock_info_[0].in_write_;
#endif  // SUPPORT_DEADLOCK_CHECKING
}

bool ReadWriteLock::InWriteByThisThread() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  thread_lock_info_mutex_.lock();
  bool in_write = thread_lock_info_[GetThisThreadId()].in_write_.load();
  thread_lock_info_mutex_.unlock();
  return in_write;
#else
  return thread_lock_info_[0].in_write_;
#endif  // SUPPORT_DEADLOCK_CHECKING
}

bool ReadWriteLock::IsLocked() { return InRead() || InWrite(); }

bool ReadWriteLock::IsLockedByThisThread() {
  return InReadByThisThread() || InWriteByThisThread();
}

void ReadWriteLock::ReadLock() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  auto thread_id = GetThisThreadId();
  std::lock_guard<std::mutex> lock(read_operation_mutex_);

  thread_lock_info_mutex_.lock();
  if (thread_lock_info_[thread_id].in_write_) {
    ZERROR << "Dead lock detected in thread " << thread_id << ".";
  }
  bool in_read = false;
  for (auto &&lock_info : thread_lock_info_) {
    if (lock_info.second.read_count_.load() > 0) {
      in_read = true;
      break;
    }
  }
  thread_lock_info_[thread_id].read_count_.fetch_add(1);
  thread_lock_info_mutex_.unlock();

  if (!in_read) {
    // If reader exist, lock once.
    resource_mutex_.lock();
  }

#else
  std::lock_guard<std::mutex> lock(read_operation_mutex_);
  thread_lock_info_[0].read_count_.fetch_add(1);
  if (thread_lock_info_[0].read_count_.load() == 1) {
    // If reader exist, lock once.
    resource_mutex_.lock();
  }
#endif  // SUPPORT_DEADLOCK_CHECKING
}

void ReadWriteLock::ReadUnlock() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  auto thread_id = GetThisThreadId();
  std::lock_guard<std::mutex> lock(read_operation_mutex_);

  thread_lock_info_mutex_.lock();
  if (thread_lock_info_[thread_id].read_count_.load() == 0) {
    thread_lock_info_mutex_.unlock();
    return;
  }

  thread_lock_info_[thread_id].read_count_.fetch_sub(1);
  bool in_read = false;
  for (auto &&lock_info : thread_lock_info_) {
    if (lock_info.second.read_count_.load() > 0) {
      in_read = true;
      break;
    }
  }
  thread_lock_info_mutex_.unlock();

  if (!in_read) {
    // If reader are all released, unlock once.
    resource_mutex_.unlock();
  }

#else
  std::lock_guard<std::mutex> lock(read_operation_mutex_);
  if (thread_lock_info_[0].read_count_.load() == 0) {
    return;
  }
  thread_lock_info_[0].read_count_.fetch_sub(1);
  if (thread_lock_info_[0].read_count_.load() == 0) {
    // If reader are all released, unlock once.
    resource_mutex_.unlock();
  }
#endif  // SUPPORT_DEADLOCK_CHECKING
}

void ReadWriteLock::WriteLock() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  auto thread_id = GetThisThreadId();
  thread_lock_info_mutex_.lock();
  if (thread_lock_info_[thread_id].in_write_.load() ||
      thread_lock_info_[thread_id].read_count_.load() > 0) {
    ZERROR << "Dead lock detected in thread " << thread_id << ".";
  }
  thread_lock_info_mutex_.unlock();

  resource_mutex_.lock();

  thread_lock_info_mutex_.lock();
  thread_lock_info_[thread_id].in_write_.store(true);
  thread_lock_info_mutex_.unlock();
#else
  resource_mutex_.lock();
  thread_lock_info_[0].in_write_.store(true);
#endif  // SUPPORT_DEADLOCK_CHECKING
}

void ReadWriteLock::WriteUnlock() {
#ifdef SUPPORT_DEADLOCK_CHECKING
  thread_lock_info_mutex_.lock();
  thread_lock_info_[GetThisThreadId()].in_write_.store(false);
  thread_lock_info_mutex_.unlock();

  resource_mutex_.unlock();
#else
  thread_lock_info_[0].in_write_.store(false);
  resource_mutex_.unlock();
#endif  // SUPPORT_DEADLOCK_CHECKING
}

ReadWriteLock::LockInfo::LockInfo() : read_count_(0), in_write_(false) {}

ReadWriteLock::LockInfo::~LockInfo() {}

ReadLocker::ReadLocker(ReadWriteLock::SPtr lock, const bool &lock_at_creation)
    : locked_(false) {
  p_lock_ = lock;
  if (lock_at_creation) {
    Lock();
  }
};

ReadLocker::~ReadLocker() { Unlock(); }

void ReadLocker::Lock() {
  if (!locked_) {
    p_lock_->ReadLock();
    locked_ = true;
  }
}

void ReadLocker::Unlock() {
  if (locked_) {
    p_lock_->ReadUnlock();
    locked_ = false;
  }
}

WriteLocker::WriteLocker(ReadWriteLock::SPtr lock, const bool &lock_at_creation)
    : locked_(false) {
  p_lock_ = lock;
  if (lock_at_creation) {
    Lock();
  }
};

WriteLocker::~WriteLocker() { Unlock(); }

void WriteLocker::Lock() {
  if (!locked_) {
    p_lock_->WriteLock();
    locked_ = true;
  }
}

void WriteLocker::Unlock() {
  if (locked_) {
    p_lock_->WriteUnlock();
    locked_ = false;
  }
}

}  // namespace zima
