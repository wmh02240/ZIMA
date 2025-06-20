/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/auto_cleaning_mode.h"
#include "zima/algorithm/line_segment_extractor.h"
#include "zima/algorithm/slam/simple_slam.h"
#include "zima/algorithm/slam/slam_base.h"
#include "zima/event/event_manager.h"
#include "zima/robot_data/local_nav_data.h"

namespace zima {

AutoCleaningMode::Config::Config() : Config(nullptr) {}

AutoCleaningMode::Config::Config(const JsonSPtr &json) : OperationMode::Config(json) {
    if (json == nullptr) {
        JsonSPtr config;
        if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_, config)) {
            // ZINFO << "Load config from global json config.";
            config_valid_ = ParseFromJson(config);
        }
    }
}

AutoCleaningMode::AutoCleaningMode(const Chassis::SPtr &chassis, const ChassisController::SPtr &chassis_controller, const SlamBase::SPtr &slam_wrapper,
                                   const EntranceType &entrance_type, const AutoCleaning::AutoCleaningInfo::SPtr &cached_cleaning_info,
                                   const bool &use_simple_slam)
    : OperationMode("Auto cleaning mode", chassis, chassis_controller, use_simple_slam), config_(new Config()), entrance_type_(entrance_type),
      cached_cleaning_info_(cached_cleaning_info), slam_wrapper_(slam_wrapper) {
    if (slam_wrapper == nullptr) {
        ZERROR << "Slam wrapper empty.";
        valid_ = false;
    }
    ZINFO << "Entrance type: " << DebugString(entrance_type);
    if ((entrance_type == EntranceType::kTotallyNewCleaning || entrance_type == EntranceType::kNewCleaningBaseOnSelectedNavData) &&
        cached_cleaning_info_ != nullptr) {
        ZERROR << "Cached cleaning info is just for resume cleaning.";
        cached_cleaning_info_.reset();
    }
    SwitchToReadyState();
    chassis_controller->RunThread();
}

AutoCleaningMode::~AutoCleaningMode() {}

AutoCleaningMode::State AutoCleaningMode::GetState() const {
    ReadLocker lock(access_);
    return state_;
};

bool AutoCleaningMode::IsReady() const {
    ReadLocker lock(access_);
    return state_ == kReady;
}

bool AutoCleaningMode::IsInitializing() const {
    ReadLocker lock(access_);
    return state_ == kInitializing;
}

bool AutoCleaningMode::IsRunning() const {
    ReadLocker lock(access_);
    return state_ == kRunning;
}

bool AutoCleaningMode::IsPaused() const {
    ReadLocker lock(access_);
    return state_ == kPaused;
}

bool AutoCleaningMode::IsHandlingException() const {
    ReadLocker lock(access_);
    return state_ == kHandlingException;
}

bool AutoCleaningMode::IsAutoFinished() const {
    ReadLocker lock(access_);
    return state_ == kAutoFinished;
};

bool AutoCleaningMode::IsStopped() const {
    ReadLocker lock(access_);
    return state_ == kStopped;
};

AutoCleaning::AutoCleaningInfo::SPtr AutoCleaningMode::GetCleaningInfo() {
    ReadLocker lock(access_);
    if (auto_cleaning_ == nullptr) {
        return nullptr;
    }

    auto cleaning_info = std::make_shared<AutoCleaning::AutoCleaningInfo>(auto_cleaning_->GetCleaningInfo());
    return cleaning_info;
}

void AutoCleaningMode::Run(const OperationData::SPtr &operation_data) {
    WriteLocker lock(access_);
    auto &event_manager = *EventManager::Instance();
    if (state_ != State::kReady && chassis_ == nullptr) {
        SwitchToReadyState();
        event_manager.PushCommonNotice(std::make_shared<FatalErrorNotice>());
        ZERROR << "Chassis pointer empty.";
    }
    if (operation_data == nullptr) {
        SwitchToReadyState();
        event_manager.PushCommonNotice(std::make_shared<FatalErrorNotice>());
        ZERROR << "Nav data pointer empty.";
    }

    switch (state_) {
    case State::kReady: {
        ReadyStateRun(operation_data);
        break;
    }
    case State::kInitializing: {
        InitializingStateRun(operation_data);
        break;
    }
    case State::kRunning: {
        RunningStateRun(operation_data);
        break;
    }
    case State::kPaused: {
        PausedStateRun(operation_data);
        break;
    }
    case State::kHandlingException: {
        HandlingExceptionStateRun(operation_data);
        break;
    }
    case State::kAutoFinished:
    case State::kStopped:
    default: {
        AutoFinishedStateRun(operation_data);
        break;
    }
    }
}

void AutoCleaningMode::Start() {
    WriteLocker lock(access_);
    start_request_ = true;
    ZGINFO << "Receive start request.";
}

void AutoCleaningMode::Stop() {
    WriteLocker lock(access_);
    stop_request_ = true;
    ZGINFO << "Receive stop request.";
}

void AutoCleaningMode::Pause() {
    WriteLocker lock(access_);
    pause_request_ = true;
    ZGINFO << "Receive pause request.";
}

void AutoCleaningMode::ReadyStateRun(const OperationData::SPtr &operation_data) {
    // Wait for event.
    if (start_request_) {
        ZINFO << "Handle start request.";
        SwitchToInitializingState();
        start_request_ = false;
        return;
    }
    if (stop_request_) {
        ZWARN << "Stop request will not be handled in this state.";
        stop_request_ = false;
    }
    if (pause_request_) {
        ZWARN << "Stop request will not be handled in this state.";
        pause_request_ = false;
    }

    ChassisController::InfoWrapper info;
    chassis_controller_->GlanceInfo(info);
    if (info.GetControllerState() == ChassisController::InfoWrapper::ControllerState::kRunning) {
        ZGINFO << "Stop current movement.";
        chassis_controller_->ForceStopMovement();
    }
}

void AutoCleaningMode::InitializingStateRun(const OperationData::SPtr &operation_data) {
    // Initialize
    if (start_request_) {
        ZWARN << "Start request will not be handled in this state.";
        start_request_ = false;
    }
    if (stop_request_) {
        ZINFO << "Handle stop request.";
        stop_request_ = false;
        SwitchToReadyState();
        return;
    }
    switch (entrance_type_) {
    case EntranceType::kNewCleaningBaseOnSelectedNavData: {
        if (pause_request_) {
            ZINFO << "Handle pause request, initialization interruptted, cancel "
                     "cleaning.";
            pause_request_ = false;
            SwitchToStoppedState(operation_data);
            break;
        }

        if (InitializeForNewCleaningBaseOnSelectedNavData(operation_data)) {
            SwitchToRunningState(operation_data);
        }
        break;
    }
    case EntranceType::kResumeCleaningWithRobotQuiescentState: {
        if (pause_request_) {
            ZINFO << "Handle pause request.";
            pause_request_ = false;
            SwitchToPauseState(operation_data);
            break;
        }

        if (InitializeForResumeCleaningWithRobotQuiescentState(operation_data)) {
            SwitchToRunningState(operation_data);
        }
        break;
    }
    case EntranceType::kResumeCleaningWithRobotActiveState: {
        if (pause_request_) {
            ZINFO << "Handle pause request.";
            pause_request_ = false;
            SwitchToPauseState(operation_data);
            break;
        }

        if (InitializeForResumeCleaningWithRobotActiveState(operation_data)) {
            SwitchToRunningState(operation_data);
        }
        break;
    }
    case EntranceType::kTotallyNewCleaning:
    default: {
        if (pause_request_) {
            ZINFO << "Handle pause request, initialization interruptted, cancel "
                     "cleaning.";
            pause_request_ = false;
            SwitchToStoppedState(operation_data);
            break;
        }
        if (InitializeForTotallyNewCleaning(operation_data)) {
            SwitchToRunningState(operation_data);
        }
        break;
    }
    }
}

void AutoCleaningMode::RunningStateRun(const OperationData::SPtr &operation_data) {
    if (start_request_) {
        ZWARN << "Start request will not be handled in this state.";
        start_request_ = false;
    }
    if (stop_request_) {
        ZINFO << "Handle stop request.";
        stop_request_ = false;
        SwitchToReadyState();
        return;
    }
    if (pause_request_) {
        ZINFO << "Handle pause request.";
        pause_request_ = false;
        SwitchToPauseState(operation_data);
        return;
    }

    auto tf = TransformManager::Instance();
    Transform robot_tf("", "");
    tf->GetTransform(tf->kWorldFrame_, tf->kRobotFrame_, robot_tf);
    MapPoint world_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    // tf->GetTransform(tf->kOdomFrame_, tf->kRobotFrame_, robot_tf);
    // MapPoint odom_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());

    // ZINFO << "World pose " << world_pose.DebugString();
    // ZINFO << "Odom pose " << world_pose.DebugString();

    operation_data->AddStep(StepPoint(world_pose));

    auto_cleaning_->ChassisSupervise(chassis_, chassis_controller_);
    if (auto_cleaning_->GetState() == AutoCleaning::State::kFinished) {
        SwitchToAutoFinishedState(operation_data);
    }
}

void AutoCleaningMode::PausedStateRun(const OperationData::SPtr &operation_data) {
    // Do nothing.
}

void AutoCleaningMode::HandlingExceptionStateRun(const OperationData::SPtr &operation_data) {}

void AutoCleaningMode::AutoFinishedStateRun(const OperationData::SPtr &operation_data) {}

void AutoCleaningMode::SwitchToReadyState() {
    // Reset all variables.
    ZGINFO << "Switch to ready state.";
    state_ = State::kReady;
    auto_cleaning_.reset();

    if (slam_wrapper_->IsRunning() && !slam_wrapper_->StopSlam()) {
        ZERROR << "Stop slam failed.";
    }
    initialize_state_ = InitializeState::kNull;
    initialize_start_time_ = 0;
    operate_on_slam_time_ = 0;
    lidar_ready_ = false;
    odom_ready_ = false;
    async_point_cloud_matcher_.reset();
    relocation_count_ = 0;
}

void AutoCleaningMode::SwitchToInitializingState() {
    // Reset initialization relative variables.
    ZGINFO << "Switch to initializing state.";
    state_ = State::kInitializing;

    initialize_state_ = InitializeState::kNull;
    initialize_start_time_ = Time::Now();
    lidar_ready_ = false;
    odom_ready_ = false;
    async_point_cloud_matcher_.reset();
    relocation_count_ = 0;
}

void AutoCleaningMode::SwitchToRunningState(const OperationData::SPtr &operation_data) {
    ZGINFO << "Switch to running state.";
    state_ = State::kRunning;
}

void AutoCleaningMode::SwitchToPauseState(const OperationData::SPtr &operation_data) {
    ZGINFO << "Switch to pause state.";
    if (state_ == State::kRunning || state_ == State::kHandlingException) {
        auto tf = TransformManager::Instance();
        Transform robot_tf("", "");
        tf->GetTransform(tf->kWorldFrame_, tf->kRobotFrame_, robot_tf);
        operation_data->SetPauseWorldPose(std::make_shared<MapPoint>(robot_tf.X(), robot_tf.Y(), robot_tf.Degree()));
        ZGINFO << "Set pause world pose: " << operation_data->GetPauseWorldPose()->DebugString();
    }
    state_ = State::kPaused;
    if (auto_cleaning_ != nullptr) {
        auto_cleaning_->Pause(chassis_, chassis_controller_);
    }

    operation_data->GetStopWatchRef().Pause();
    slam_wrapper_->PauseSlam();
    operate_on_slam_time_ = Time::Now();
}

void AutoCleaningMode::SwitchToAutoFinishedState(const OperationData::SPtr &operation_data) {
    ZGINFO << "Switch to auto finished state.";
    state_ = State::kAutoFinished;
    operation_data->SetOperationResult(OperationData::OperationResult::kFinishAutoCleaning);
    operation_data->GetStopWatchRef().Stop();
    operate_on_slam_time_ = Time::Now();
}

void AutoCleaningMode::SwitchToStoppedState(const OperationData::SPtr &operation_data) {
    ZGINFO << "Switch to stopped state.";
    state_ = State::kStopped;
    operation_data->SetOperationResult(OperationData::OperationResult::kStopped);
    operation_data->GetStopWatchRef().Stop();
    operate_on_slam_time_ = Time::Now();
}

bool AutoCleaningMode::InitializeForTotallyNewCleaning(const OperationData::SPtr &operation_data) {
    switch (initialize_state_) {
    case InitializeState::kNull: {
        auto_cleaning_.reset(new AutoCleaning(operation_data));
        operation_data->GetStopWatchRef().Start();
        operation_data->SetSlamMapFileName(LocalNavDataManager::Instance()->GetNavDataPath() +
                                           std::to_string(static_cast<time_t>(floor(operation_data->GetStartTime()))) + LocalNavData::kSlamFileSuffix_);

        auto &event_manager = *EventManager::Instance();
        event_manager.PushCommonNotice(std::make_shared<StartCleaningNotice>());

        initialize_state_ = InitializeState::kInitForSensor;
        ZGINFO << "Switch to state kInitForSensor";
        break;
    }
    case InitializeState::kInitForSensor: {
        // Currently does not need to init sensor from here.
        InitializeForSensor(operation_data);
        break;
    }
    case InitializeState::kWaitForLidarAndOdomData: {
        if (WaitForSensor(operation_data)) {
            initialize_state_ = InitializeState::kInitForSlam;
            ZGINFO << "Switch to state kInitForSlam";
        }

        break;
    }
    case InitializeState::kInitForSlam: {
        auto &tf = *TransformManager::Instance();
        MapPoint current_pose(0, 0);
        Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_, current_pose.X(), current_pose.Y(), current_pose.Degree());
        tf.UpdateTransform(new_odom_tf);
        Transform new_correction_tf(tf.kWorldFrame_, tf.kOdomFrame_, 0, 0, 0);
        tf.UpdateTransform(new_correction_tf);

        MapPoint start_pose;
        auto point_cloud_in_chassis_frame = chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInChassisFrame();
        if (point_cloud_in_chassis_frame != nullptr) {
            double most_valued_degree = 0;
            PointCloudLineSegmentExtractor::GetMostValuedLineDegree(point_cloud_in_chassis_frame, most_valued_degree);

            start_pose.SetDegree(-1 * most_valued_degree);
        }

        // start_pose.SetY(-1 / 2 *
        //                 operation_data->GetNavMapConstRef()->GetResolution() *
        //                 0.8);
        operation_data->SetStartPoint(start_pose);
        // Start slam.
        if (slam_wrapper_->StartSlam(start_pose, operation_data->GetSlamMapFileName(), "")) {
            initialize_state_ = InitializeState::kWaitForSlam;
            operate_on_slam_time_ = Time::Now();
            ZGINFO << "Switch to state kWaitForSlam";
        } else {
            initialize_state_ = InitializeState::kInitForSlam;
            ZGINFO << "Re-enter kInitForSlam";
        }

        break;
    }
    case InitializeState::kWaitForSlam: {
        auto map = slam_wrapper_->GetSlamMap();
        if (map != nullptr) {
            initialize_state_ = InitializeState::kInitFinish;
            ZGINFO << "Switch to state kInitFinish";
        }

        break;
    }
    case InitializeState::kInitFinish:
    default: {
        ZGINFO << "Initialize finish.";
        return true;
    }
    }
    return false;
}

bool AutoCleaningMode::InitializeForNewCleaningBaseOnSelectedNavData(const OperationData::SPtr &operation_data) {
    switch (initialize_state_) {
    case InitializeState::kNull: {
        auto_cleaning_.reset(new AutoCleaning(operation_data));
        operation_data->GetStopWatchRef().Start();

        auto &event_manager = *EventManager::Instance();
        event_manager.PushCommonNotice(std::make_shared<StartCleaningNotice>());

        initialize_state_ = InitializeState::kInitForSensor;
        ZGINFO << "Switch to state kInitForSensor";
        break;
    }
    case InitializeState::kInitForSensor: {
        // Currently does not need to init sensor from here.
        InitializeForSensor(operation_data);
        break;
    }
    case InitializeState::kWaitForLidarAndOdomData: {
        if (WaitForSensor(operation_data)) {
            initialize_state_ = InitializeState::kInitForRelocation;
            ZGINFO << "Switch to state kInitForRelocation";
        }
        break;
    }
    case InitializeState::kInitForRelocation: {
        relocation_count_++;
        ZINFO << "Relocation for " << static_cast<int>(relocation_count_) << " time.";
        if (relocation_count_ > config_->relocation_count_max_) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<RelocationFailedNotice>());
            SwitchToStoppedState(operation_data);
            ZERROR << "Relocation failed.";
            return false;
        } else {
            auto map = operation_data->GetRawSlamValueGridMap2DRef();
            if (FLAGS_debug_enable) {
                map->Print(__FILE__, __FUNCTION__, __LINE__);
            }
            operation_data->GetOptimizedSlamValueGridMap2DRef()->Print(__FILE__, __FUNCTION__, __LINE__);
            auto point_cloud_in_chassis_frame = chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInChassisFrame();

            if (relocation_count_ == 1) {
                auto &event_manager = *EventManager::Instance();
                event_manager.PushCommonNotice(std::make_shared<StartRelocationNotice>());
                ZGINFO << "Relocation started.";
                double most_valued_degree = 0;
                PointCloudLineSegmentExtractor::GetMostValuedLineDegree(point_cloud_in_chassis_frame, most_valued_degree);

                MapPoint init_pose(0, 0, -1 * most_valued_degree);
                DynamicMapPointBound bound_in_map_frame(map->GetDataPointBound());
                PointCloudMatcher::BABSearchParameter parameter(map, init_pose, bound_in_map_frame, config_->optimized_global_search_config_->degree_range_,
                                                                config_->optimized_global_search_config_->degree_step_,
                                                                config_->optimized_global_search_config_->search_depth_,
                                                                config_->optimized_global_search_config_->min_score_, true);
                ZGINFO << "Match with optimized degree " << FloatToString(init_pose.Degree(), 2) << " for first match.";
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            } else {
                MapPoint init_pose(0, 0, 0);
                DynamicMapPointBound bound_in_map_frame(map->GetDataPointBound());
                PointCloudMatcher::BABSearchParameter parameter(map, init_pose, bound_in_map_frame, config_->global_search_config_->degree_range_,
                                                                config_->global_search_config_->degree_step_, config_->global_search_config_->search_depth_,
                                                                config_->global_search_config_->min_score_);

                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            }
        }
        initialize_state_ = InitializeState::kWaitForRelocation;
        ZGINFO << "Switch to state kWaitForRelocation";

        break;
    }
    case InitializeState::kWaitForRelocation: {
        if (async_point_cloud_matcher_ == nullptr) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<RelocationFailedNotice>());
            SwitchToReadyState();
            ZERROR << "Relocation matcher released.";
            return false;
        }
        PointCloudMatcher::MatchResult match_result;
        if (async_point_cloud_matcher_->GetMatchResult(match_result)) {
            if (match_result.score_ > async_point_cloud_matcher_->GetParameter().min_score_) {
                auto &event_manager = *EventManager::Instance();
                event_manager.PushCommonNotice(std::make_shared<RelocationSucceedNotice>());
                ZINFO << "Relocation succeed.";

                initialize_state_ = InitializeState::kInitForSlam;
                ZGINFO << "Switch to state kInitForSlam";
            } else {
                initialize_state_ = InitializeState::kChangePoseForRelocation;
                ZGINFO << "Switch to state kChangePoseForRelocation";
            }
        }
        break;
    }
    case InitializeState::kChangePoseForRelocation: {
        initialize_state_ = InitializeState::kInitForRelocation;
        ZGINFO << "Switch to state kInitForRelocation";

        break;
    }
    case InitializeState::kInitForSlam: {
        PointCloudMatcher::MatchResult match_result;
        if (!async_point_cloud_matcher_->GetMatchResult(match_result)) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<RelocationFailedNotice>());
            SwitchToReadyState();
            ZERROR << "Relocation matcher result released.";
            return false;
        }
        async_point_cloud_matcher_.reset();

        auto &tf = *TransformManager::Instance();
        MapPoint current_pose(match_result.pose_);
        Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_, current_pose.X(), current_pose.Y(), current_pose.Degree());
        tf.UpdateTransform(new_odom_tf);
        Transform new_correction_tf(tf.kWorldFrame_, tf.kOdomFrame_, 0, 0, 0);
        tf.UpdateTransform(new_correction_tf);

        // Start slam.
        if (use_simple_slam_) {
            auto slam = dynamic_pointer_cast<SimpleSlam>(slam_wrapper_);
            slam->UpdateProbabilityMap(operation_data->GetProbabilityIndexGridMap2DRef());
            if (slam->GetProbabilityMap() == nullptr) {
                ZERROR << "Missing map.";
            } else {
                // slam->GetProbabilityMap()->Print(__FILE__, __FUNCTION__, __LINE__);
            }
        }

        if (FLAGS_debug_enable) {
            operation_data->GetNavMapConstRef()->GetRoomLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
        }

        operation_data->SetStartPoint(current_pose);
        if (slam_wrapper_->StartSlam(current_pose, operation_data->GetSlamMapFileName(), operation_data->GetSlamMapFileName())) {
            initialize_state_ = InitializeState::kWaitForSlam;
            operate_on_slam_time_ = Time::Now();
            ZGINFO << "Switch to state kWaitForSlam";
        } else {
            initialize_state_ = InitializeState::kInitForSlam;
            ZGINFO << "Re-enter kInitForSlam";
        }

        break;
    }
    case InitializeState::kWaitForSlam: {
        auto map = slam_wrapper_->GetSlamMap();
        if (map != nullptr) {
            initialize_state_ = InitializeState::kInitFinish;
            ZGINFO << "Switch to state kInitFinish";
        }

        break;
    }
    case InitializeState::kInitFinish:
    default: {
        ZGINFO << "Initialize finish.";
        return true;
    }
    }
    return false;
}

bool AutoCleaningMode::InitializeForResumeCleaningWithRobotQuiescentState(const OperationData::SPtr &operation_data) {
    switch (initialize_state_) {
    case InitializeState::kNull: {
        if (cached_cleaning_info_ != nullptr) {
            ZGINFO << "Resume auto cleaning for cached info.";
        }
        auto_cleaning_.reset(new AutoCleaning(operation_data, cached_cleaning_info_));
        auto &event_manager = *EventManager::Instance();
        event_manager.PushCommonNotice(std::make_shared<ResumeCleaningNotice>());

        initialize_state_ = InitializeState::kInitForSensor;
        ZGINFO << "Switch to state kInitForSensor";
        break;
    }
    case InitializeState::kInitForSensor: {
        // Currently does not need to init sensor from here.
        InitializeForSensor(operation_data);
        break;
    }
    case InitializeState::kWaitForLidarAndOdomData: {
        if (WaitForSensor(operation_data)) {
            initialize_state_ = InitializeState::kInitForRelocation;
            ZGINFO << "Switch to state kInitForRelocation";
        }
        break;
    }
    case InitializeState::kInitForRelocation: {
        relocation_count_++;
        ZINFO << "Relocation for " << static_cast<int>(relocation_count_) << " time.";
        if (relocation_count_ > config_->relocation_count_max_) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<RelocationFailedNotice>());
            SwitchToStoppedState(operation_data);
            ZERROR << "Relocation failed.";
            return false;
        } else if (relocation_count_ == 1) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<StartRelocationNotice>());
            ZGINFO << "Relocation started.";
        }

        auto point_cloud_in_chassis_frame = chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInChassisFrame();

        auto map = operation_data->GetRawSlamValueGridMap2DRef();

        if (FLAGS_debug_enable) {
            map->Print(__FILE__, __FUNCTION__, __LINE__);
        }
        operation_data->GetOptimizedSlamValueGridMap2DRef()->Print(__FILE__, __FUNCTION__, __LINE__);
        auto pause_world_pose = operation_data->GetPauseWorldPose();
        if (pause_world_pose != nullptr) {
            DynamicMapPointBound bound_near_pause_pose_in_map_frame(
                pause_world_pose->X() - config_->small_range_linear_range_, pause_world_pose->X() + config_->small_range_linear_range_,
                pause_world_pose->Y() - config_->small_range_linear_range_, pause_world_pose->Y() + config_->small_range_linear_range_);

            if (relocation_count_ == 1) {
                double most_valued_degree = 0;
                PointCloudLineSegmentExtractor::GetMostValuedLineDegree(point_cloud_in_chassis_frame, most_valued_degree);
                pause_world_pose->SetDegree(-1 * most_valued_degree);
                PointCloudMatcher::BABSearchParameter parameter(
                    map, *pause_world_pose, bound_near_pause_pose_in_map_frame, config_->optimized_small_range_search_config_->degree_range_,
                    config_->optimized_small_range_search_config_->degree_step_, config_->optimized_small_range_search_config_->search_depth_,
                    config_->optimized_small_range_search_config_->min_score_, true);
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            } else if (relocation_count_ == 2) {
                PointCloudMatcher::BABSearchParameter parameter(
                    map, *pause_world_pose, bound_near_pause_pose_in_map_frame, config_->small_range_search_config_->degree_range_,
                    config_->small_range_search_config_->degree_step_, config_->small_range_search_config_->search_depth_,
                    config_->small_range_search_config_->min_score_);
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            } else if (relocation_count_ == 3) {
                MapPoint init_pose(0, 0, 0);
                double most_valued_degree = 0;
                PointCloudLineSegmentExtractor::GetMostValuedLineDegree(point_cloud_in_chassis_frame, most_valued_degree);
                init_pose.SetDegree(-1 * most_valued_degree);
                DynamicMapPointBound bound_in_map_frame(map->GetDataPointBound());
                PointCloudMatcher::BABSearchParameter parameter(map, init_pose, bound_in_map_frame, config_->optimized_global_search_config_->degree_range_,
                                                                config_->optimized_global_search_config_->degree_step_,
                                                                config_->optimized_global_search_config_->search_depth_,
                                                                config_->optimized_global_search_config_->min_score_, true);
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            } else {
                MapPoint init_pose(0, 0, 0);
                DynamicMapPointBound bound_in_map_frame(map->GetDataPointBound());
                PointCloudMatcher::BABSearchParameter parameter(map, init_pose, bound_in_map_frame, config_->global_search_config_->degree_range_,
                                                                config_->global_search_config_->degree_step_, config_->global_search_config_->search_depth_,
                                                                config_->global_search_config_->min_score_);
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            }
        } else {
            if (relocation_count_ == 1) {
                MapPoint init_pose(0, 0, 0);
                double most_valued_degree = 0;
                PointCloudLineSegmentExtractor::GetMostValuedLineDegree(point_cloud_in_chassis_frame, most_valued_degree);
                init_pose.SetDegree(-1 * most_valued_degree);
                DynamicMapPointBound bound_in_map_frame(map->GetDataPointBound());
                PointCloudMatcher::BABSearchParameter parameter(map, init_pose, bound_in_map_frame, config_->optimized_global_search_config_->degree_range_,
                                                                config_->optimized_global_search_config_->degree_step_,
                                                                config_->optimized_global_search_config_->search_depth_,
                                                                config_->optimized_global_search_config_->min_score_, true);
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            } else {
                MapPoint init_pose(0, 0, 0);
                DynamicMapPointBound bound_in_map_frame(map->GetDataPointBound());
                PointCloudMatcher::BABSearchParameter parameter(map, init_pose, bound_in_map_frame, config_->global_search_config_->degree_range_,
                                                                config_->global_search_config_->degree_step_, config_->global_search_config_->search_depth_,
                                                                config_->global_search_config_->min_score_);
                async_point_cloud_matcher_.reset(new AsyncPointCloudMatcher("Async point cloud match thread", map, point_cloud_in_chassis_frame, parameter));
            }
        }
        initialize_state_ = InitializeState::kWaitForRelocation;
        ZGINFO << "Switch to state kWaitForRelocation";

        break;
    }
    case InitializeState::kWaitForRelocation: {
        if (async_point_cloud_matcher_ == nullptr) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<RelocationFailedNotice>());
            SwitchToReadyState();
            ZERROR << "Relocation matcher released.";
            return false;
        }
        PointCloudMatcher::MatchResult match_result;
        if (async_point_cloud_matcher_->GetMatchResult(match_result)) {
            if (match_result.score_ > async_point_cloud_matcher_->GetParameter().min_score_) {
                auto &event_manager = *EventManager::Instance();
                event_manager.PushCommonNotice(std::make_shared<RelocationSucceedNotice>());
                ZINFO << "Relocation succeed.";

                initialize_state_ = InitializeState::kInitForSlam;
                ZGINFO << "Switch to state kInitForSlam";
            } else {
                initialize_state_ = InitializeState::kChangePoseForRelocation;
                ZGINFO << "Switch to state kChangePoseForRelocation";
            }
        }
        break;
    }
    case InitializeState::kChangePoseForRelocation: {
        initialize_state_ = InitializeState::kInitForRelocation;
        ZGINFO << "Switch to state kInitForRelocation";

        break;
    }
    case InitializeState::kInitForSlam: {
        PointCloudMatcher::MatchResult match_result;
        if (!async_point_cloud_matcher_->GetMatchResult(match_result)) {
            auto &event_manager = *EventManager::Instance();
            event_manager.PushCommonNotice(std::make_shared<RelocationFailedNotice>());
            SwitchToReadyState();
            ZERROR << "Relocation matcher result released.";
            return false;
        }
        async_point_cloud_matcher_.reset();

        auto &tf = *TransformManager::Instance();
        MapPoint current_pose(match_result.pose_);

        Transform current_correction_tf(tf.kWorldFrame_, tf.kOdomFrame_, 0, 0, 0);
        tf.GetTransform(tf.kWorldFrame_, tf.kOdomFrame_, current_correction_tf);
        double new_odom_x, new_odom_y;
        Transform::CoordinateTransformationAB(current_pose.X(), current_pose.Y(), current_correction_tf.X(), current_correction_tf.Y(),
                                              DegreesToRadians(current_correction_tf.Degree()), new_odom_x, new_odom_y);
        MapPoint new_odom(new_odom_x, new_odom_y, NormalizeDegree(current_pose.Degree() - current_correction_tf.Degree()));
        ZGINFO << "New odom: " << new_odom.DebugString();
        Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_, new_odom.X(), new_odom.Y(), new_odom.Degree());
        tf.UpdateTransform(new_odom_tf);

        // Resume slam.
        if (slam_wrapper_->ResumeSlam(current_pose)) {
            initialize_state_ = InitializeState::kWaitForSlam;
            operate_on_slam_time_ = Time::Now();
            ZGINFO << "Switch to state kWaitForSlam";
        } else {
            initialize_state_ = InitializeState::kInitForSlam;
            ZGINFO << "Re-enter kInitForSlam";
        }

        break;
    }
    case InitializeState::kWaitForSlam: {
        if (Time::Now() - operate_on_slam_time_ > 1) {
            auto map = slam_wrapper_->GetSlamMap();
            if (map != nullptr) {
                initialize_state_ = InitializeState::kInitFinish;
                ZGINFO << "Switch to state kInitFinish";
            }
        }

        break;
    }
    case InitializeState::kInitFinish:
    default: {
        ZGINFO << "Initialize finish.";
        operation_data->GetStopWatchRef().Resume();
        return true;
    }
    }
    return false;
}

bool AutoCleaningMode::InitializeForResumeCleaningWithRobotActiveState(const OperationData::SPtr &operation_data) {
    ZERROR << "Don't run here for now.";
    return false;
}

bool AutoCleaningMode::InitializeForSensor(const OperationData::SPtr &operation_data) {
    lidar_ready_ = false;
    odom_ready_ = false;
    initialize_state_ = InitializeState::kWaitForLidarAndOdomData;
    ZGINFO << "Switch to state kWaitForLidarAndOdomData";
    return true;
}

bool AutoCleaningMode::WaitForSensor(const OperationData::SPtr &operation_data) {
    auto lidar = chassis_->GetLidar(chassis_->kLidar_);
    if (lidar->CheckFresh(0.5)) {
        if (!lidar_ready_) {
            ZINFO << "Lidar ready.";
        }
        lidar_ready_ = true;
    }

    if (chassis_->CheckMergedOdomDataFresh(0.2)) {
        if (!odom_ready_) {
            ZINFO << "Merged odom ready.";
        }
        odom_ready_ = true;
    }

    if (lidar_ready_ && odom_ready_) {
        return true;
    }

    static const double kInitializeLimitTime = 5;
    if (Time::Now() - initialize_start_time_ > kInitializeLimitTime) {
        SwitchToStoppedState(operation_data);
        auto &event_manager = *EventManager::Instance();
        event_manager.PushCommonNotice(std::make_shared<FatalErrorNotice>());
        ZERROR << "Sensor not available.";
    }
    return false;
}

} // namespace zima
