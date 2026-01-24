/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/event/common_notice.h"

namespace zima {

const std::string StartCleaningNotice::kLabel_ = "Start cleaning";
const std::string StartScanningNotice::kLabel_ = "Start scanning";
const std::string StartRelocationNotice::kLabel_ = "Start relocation";
const std::string RelocationSucceedNotice::kLabel_ = "Relocation succeed";
const std::string RelocationFailedNotice::kLabel_ = "Relocation failed";
const std::string PauseNotice::kLabel_ = "Pause";
const std::string ResumeCleaningNotice::kLabel_ = "Resume cleaning";
const std::string ResumeScanningNotice::kLabel_ = "Resume scanning";
const std::string FinishCleaningNotice::kLabel_ = "Finish cleaning";
const std::string FinishScanningNotice::kLabel_ = "Finish scanning";
const std::string FatalErrorNotice::kLabel_ = "Fatal error";

}  // namespace zima
