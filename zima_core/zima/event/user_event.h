/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_USER_EVENT_H
#define ZIMA_USER_EVENT_H

#include <list>

#include "zima/event/event_base.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class UserEvent : public EventBase {
 public:
  UserEvent() = delete;
  UserEvent(const std::string& label) : EventBase(label) {}
  ~UserEvent() = default;

  using SPtr = std::shared_ptr<UserEvent>;
  using SPtrList = std::list<SPtr>;

  // For dynamic_cast.
  virtual void Void() {};
};

class GeneralOperationEvent : public UserEvent {
 public:
  GeneralOperationEvent() : UserEvent(kLabel_) {}
  ~GeneralOperationEvent() = default;

  static const std::string kLabel_;
};

class StartCleaningEvent : public UserEvent {
 public:
  StartCleaningEvent() : UserEvent(kLabel_) {}
  ~StartCleaningEvent() = default;

  static const std::string kLabel_;
};

class PauseEvent : public UserEvent {
 public:
  PauseEvent() : UserEvent(kLabel_) {}
  ~PauseEvent() = default;

  static const std::string kLabel_;
};

class ResumeCleaningEvent : public UserEvent {
 public:
  ResumeCleaningEvent() : UserEvent(kLabel_) {}
  ~ResumeCleaningEvent() = default;

  static const std::string kLabel_;
};

class StopCleaningEvent : public UserEvent {
 public:
  StopCleaningEvent() : UserEvent(kLabel_) {}
  ~StopCleaningEvent() = default;

  static const std::string kLabel_;
};

class VirtualWallBlockAreaEvent : public UserEvent {
 public:
  class Operation {
   public:
    enum DataType {
      kVirtualWall,
      kBlockArea,
    };

    enum OperationType {
      kAdd,
      kRemove,
      kUpdate,
    };
    Operation() = delete;
    Operation(const DataType& data_type, const OperationType& operation_type,
              const NavData::VirtualWall::SPtr& virtual_wall,
              const NavData::BlockArea::SPtr& block_area, const uint8_t& index)
        : data_type_(data_type),
          operation_type_(operation_type),
          virtual_wall_(virtual_wall),
          block_area_(block_area),
          index_(index) {}

    using List = std::list<Operation>;

    const DataType data_type_;
    const OperationType operation_type_;
    const NavData::VirtualWall::SPtr virtual_wall_;
    const NavData::BlockArea::SPtr block_area_;
    const uint8_t index_;
  };

  VirtualWallBlockAreaEvent() = delete;
  VirtualWallBlockAreaEvent(const Operation::List& operations)
      : UserEvent(kLabel_), operations_(operations) {}
  ~VirtualWallBlockAreaEvent() = default;

  static Operation::List DemoVirtualWallBlockAreaOperation();
  static bool CreateDemoOperationTemplateFile();

  static const std::string kLabel_;
  static const std::string kFileDir_;
  static const std::string kAddVirtualWallSuffix_;
  static const std::string kRemoveVirtualWallSuffix_;
  static const std::string kUpdateVirtualWallSuffix_;
  static const std::string kAddBlockAreaSuffix_;
  static const std::string kRemoveBlockAreaSuffix_;
  static const std::string kUpdateBlockAreaSuffix_;

  Operation::List operations_;
};

class LocalNavDataEvent : public UserEvent {
 public:
  enum OperationType {
    kSelect,
    kUnSelect,
    kDelete,
  };

  LocalNavDataEvent() = delete;
  LocalNavDataEvent(const OperationType& operation_type, const uint32_t& index)
      : UserEvent(kLabel_), operation_type_(operation_type), index_(index) {}
  ~LocalNavDataEvent() = default;

  using SPtr = std::shared_ptr<LocalNavDataEvent>;

  static const std::string kLabel_;
  const OperationType operation_type_;
  const uint32_t index_;
};


class KeyboardEvent : public UserEvent {
 public:
  KeyboardEvent() = delete;
  KeyboardEvent(const char& key) : UserEvent(kLabel_), key_(key) {}
  ~KeyboardEvent() = default;

  using SPtr = std::shared_ptr<KeyboardEvent>;

  static const std::string kLabel_;
  const char key_;
};

}  // namespace zima

#endif  // ZIMA_USER_NOTICE_H
