/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_LOCK_H
#define ZIMA_LOCK_H

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

namespace zima {
using namespace std;

// This lock should be define in resources.
// This lock can only be locked by class ReadLocker/WriteLocker, for ensuring
// every lock would be safely unlock.
class ReadWriteLock {
 public:
  ReadWriteLock();
  ~ReadWriteLock();

  using SPtr = std::shared_ptr<ReadWriteLock>;

  bool InRead();
  bool InReadByThisThread();
  uint16_t ReaderCount();
  uint16_t ReaderCountForThisThread();
  bool InWrite();
  bool InWriteByThisThread();
  bool IsLocked();
  bool IsLockedByThisThread();

 private:
  friend class ReadLocker;
  friend class WriteLocker;

  void ReadLock();
  void ReadUnlock();
  void WriteLock();
  void WriteUnlock();
  std::mutex thread_lock_info_mutex_;
  std::mutex read_operation_mutex_;
  std::mutex resource_mutex_;

  class LockInfo {
   public:
    LockInfo();
    ~LockInfo();

    atomic_int16_t read_count_;
    atomic_bool in_write_;
  };

  std::map<int, LockInfo> thread_lock_info_;
};

class ReadLocker {
 public:
  ReadLocker() = delete;
  ReadLocker(ReadWriteLock::SPtr lock, const bool &lock_at_creation = true);
  ~ReadLocker();

  void Lock();
  void Unlock();

 private:
  ReadWriteLock::SPtr p_lock_;
  bool locked_;
};

class WriteLocker {
 public:
  WriteLocker() = delete;
  WriteLocker(ReadWriteLock::SPtr lock, const bool &lock_at_creation = true);
  ~WriteLocker();

  void Lock();
  void Unlock();

 private:
  ReadWriteLock::SPtr p_lock_;
  bool locked_;
};

}  // namespace zima

#endif  // ZIMA_LOCK_H
