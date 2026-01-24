/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_COMMON_NOTICE_H
#define ZIMA_COMMON_NOTICE_H

#include <list>

#include "zima/event/event_base.h"

namespace zima {

class CommonNotice : public EventBase {
 public:
  CommonNotice() = delete;
  CommonNotice(const std::string& label) : EventBase(label) {}
  ~CommonNotice() = default;

  using SPtr = std::shared_ptr<CommonNotice>;
  using SPtrList = std::list<SPtr>;

  // For dynamic_cast.
  virtual void Void() {};
};

class StartCleaningNotice : public CommonNotice {
 public:
  StartCleaningNotice() : CommonNotice(kLabel_) {}
  ~StartCleaningNotice() = default;

  static const std::string kLabel_;
};

class StartScanningNotice : public CommonNotice {
 public:
  StartScanningNotice() : CommonNotice(kLabel_) {}
  ~StartScanningNotice() = default;

  static const std::string kLabel_;
};


class StartRelocationNotice : public CommonNotice {
 public:
  StartRelocationNotice() : CommonNotice(kLabel_) {}
  ~StartRelocationNotice() = default;

  static const std::string kLabel_;
};

class RelocationSucceedNotice : public CommonNotice {
 public:
  RelocationSucceedNotice() : CommonNotice(kLabel_) {}
  ~RelocationSucceedNotice() = default;

  static const std::string kLabel_;
};

class RelocationFailedNotice : public CommonNotice {
 public:
  RelocationFailedNotice() : CommonNotice(kLabel_) {}
  ~RelocationFailedNotice() = default;

  static const std::string kLabel_;
};

class PauseNotice : public CommonNotice {
 public:
  PauseNotice() : CommonNotice(kLabel_) {}
  ~PauseNotice() = default;

  static const std::string kLabel_;
};

class ResumeCleaningNotice : public CommonNotice {
 public:
  ResumeCleaningNotice() : CommonNotice(kLabel_) {}
  ~ResumeCleaningNotice() = default;

  static const std::string kLabel_;
};

class ResumeScanningNotice : public CommonNotice {
 public:
  ResumeScanningNotice() : CommonNotice(kLabel_) {}
  ~ResumeScanningNotice() = default;

  static const std::string kLabel_;
};

class FinishCleaningNotice : public CommonNotice {
 public:
  FinishCleaningNotice() : CommonNotice(kLabel_) {}
  ~FinishCleaningNotice() = default;

  static const std::string kLabel_;
};

class FinishScanningNotice : public CommonNotice {
 public:
  FinishScanningNotice() : CommonNotice(kLabel_) {}
  ~FinishScanningNotice() = default;

  static const std::string kLabel_;
};

class FatalErrorNotice : public CommonNotice {
 public:
  FatalErrorNotice() : CommonNotice(kLabel_) {}
  ~FatalErrorNotice() = default;

  static const std::string kLabel_;
};

}  // namespace zima

#endif  // ZIMA_COMMON_NOTICE_H
