/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 * 系统级仿真、完整的机器人清扫、导航模式流程仿真
 */

#include "gazebo/zima_gazebo_node.h"
#include "zima/event/event_manager.h"
#include "zima/grid_map/map_util.h"
#include "zima/hal/system/cpu.h"
#include "zima/hal/system/keyboard_listener.h"
#include "zima/mode/mode_switcher.h"
#include "zima/zima_base_version.h"
#include "zima/zima_core_version.h"
#include "zima_ros/util.h"

DEFINE_bool(use_simple_slam, true, "Indicator for using simple slam.");

namespace zima_ros {

using namespace zima;

void ZimaGazeboNode::Run(const ZimaThreadWrapper::ThreadParam &param) {
    ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
    auto thread_manager = ZimaThreadManager::Instance();
    if (param.bind_cpu_id_ >= 0) {
        thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_); // 将线程绑定到制定cpu核心
    }

    ChassisController::SPtr chassis_controller(new ChassisController(chassis_)); // 初始化底盘控制器
    ModeSwitcher::SPtr mode_switcher_(new ModeSwitcher());                       // 初始化模式切换器
    ModeEventWrapperBase::SPtr mode_wrapper = nullptr;

    // 仿真时间同步
    if (use_sim_time_) {
        ZINFO << "Wait for sim time";
        auto now = Time::SystemNow();
        while (!shutdown_request_ && !sim_time_received_.load()) {
            thread_manager->UpdateThreadCycle(param.thread_name_);
            Time::SleepSec(0.05);
            if (Time::SystemNow() - now > 3) {
                ZINFO << "Wait for sim time";
                now = Time::SystemNow();
            }
            continue;
        }
        if (!shutdown_request_) {
            ZINFO << "Sim time received";
        }
    }

    Timer process_map_timer("process map timer", IsRunningOnARM() ? 10 : 3, true, true);
    uint32_t map_seq = 0;
    const float duration = 0.02;

    while (!shutdown_request_) {
        thread_manager->UpdateThreadCycle(param.thread_name_);

        {
            WriteLocker lock(operation_data_access_);
            bool finish_cleaning = !mode_switcher_->Run(mode_wrapper, chassis_controller, chassis_, operation_data_, slam_wrapper_);
            if (finish_cleaning) {
                break;
            }
        }

        GazeboPublishChassisControl(); // 发布底盘控制命令到gazebo

        {
            ReadLocker lock(operation_data_access_); // 先加读锁，获取operation_data_指针
            auto operation_data = operation_data_;
            lock.Unlock(); // 立即释放锁，防止长时间持锁
            if (operation_data != nullptr) {
                // 外部slam模式且已收到外部slam地图
                if (!use_simple_slam_ && slam_map_received_) {
                    ReadLocker lock(cache_slam_occupancy_grid_map_access_); // 加锁访问cache_slam_occupancy_grid_map_并检查地图序号是否更新
                    if (cache_slam_occupancy_grid_map_->header.seq != map_seq) {
                        map_seq = cache_slam_occupancy_grid_map_->header.seq; // 更新地图序号
                        // 调用OccupancyGridToSlamValueGridMap将ros的OccupancyGrid转换为内部slam地图格式
                        auto slam_grid_map = zima_ros::OccupancyGridToSlamValueGridMap("slam map", cache_slam_occupancy_grid_map_);
                        // 将新地图推送到操作数据，供后续导航/路径规划等模块使用
                        operation_data->PushNewSlamValueMap(slam_grid_map);
                    }
                }
                // slam正在运行且定时器到期，防止频繁处理
                if (use_simple_slam_ && slam_wrapper_->IsRunning() && process_map_timer.TimeUp()) {
                    auto slam_grid_map = slam_wrapper_->GetSlamMap();
                    if (slam_grid_map != nullptr) {
                        operation_data->PushNewSlamValueMap(slam_grid_map);
                        process_map_timer.Reset(); // 重置定时器
                    }
                }
            }
        }
        Time::SleepSec(duration); // 控制循环频率
    }

    chassis_controller->StopThread();

    Time::SleepMSec(300);
    ros::shutdown();

    ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
    thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__, __LINE__);
    ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

} // namespace zima_ros

int main(int argc, char **argv) {
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_debug_enable = false;
    FLAGS_use_simple_slam = true;
    FLAGS_run_with_half_scan = false;

    ZINFO << zima::GetBaseVersionInfo();
    ZINFO << zima::GetCoreVersionInfo();

    // 加载全局配置json信息
    zima::JsonSPtr global_config(nullptr);
    if (!zima::GlobalJsonConfig::Instance()->GetGlobalConfig(global_config)) {
        ZERROR << "Missing config.";
        return -1;
    }

    // 初始化一个ros节点
    ros::init(argc, argv, "zima_auto_cleaning_mode_gazebo_test");

    // 键盘监听器，按键事件通过回调推送到事件管理器（用于控制机器人或模式切换）
    zima::KeyboardListener::CallBackFunc cb = [&](const char &key) -> void {
        auto &event_manager = *zima::EventManager::Instance();
        event_manager.PushUserEvent(std::make_shared<zima::KeyboardEvent>(key));
    };
    zima::KeyboardListener keyboard_listener(cb);
    keyboard_listener.Start();

    zima_ros::ZimaGazeboNode test(true);
    // 通过 ZimaThreadManager 启动多个线程，每个线程负责不同的 ROS
    // 功能，且各国线程之间通过共享数据结构(slam_wrapper_、operation_data_等)和线程安全机制(读写锁)进行协作
    auto thread_manager = zima::ZimaThreadManager::Instance();

    // 作用：处理 ROS 订阅、服务、回调等事件，保证节点能够及时响应传感器的输入、控制指令
    // 协作：为其他线程提供消息传递和事件触发的基础环境
    zima::ZimaThreadWrapper::ThreadParam ros_thread_param("Ros spin thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2, 1);
    thread_manager->RegisterThread(std::thread(&zima_ros::ZimaNode::ROSSpinThread, &test, ros_thread_param), ros_thread_param);

    // 作用：处理 TF 坐标变换相关任务、维护机器人各个部件之间的空间关系、例如底盘、激光、地图，为感知、导航模块提供实时坐标变换信息
    // 协作：为传感器数据发布、可视化等线程提供准确的坐标变换支持
    zima::ZimaThreadWrapper::ThreadParam tf_thread_param("Tf thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2, 1);
    thread_manager->RegisterThread(std::thread(&zima_ros::ZimaNode::TfThread, &test, tf_thread_param), tf_thread_param);

    // 作用：发布传感器数据（如里程计、激光等），将传感器状态实时发布到ros话题，共其他模块使用，例如slam、可视化使用
    // 协作：为slam、可视化等线程提供实时的传感器输入
    zima::ZimaThreadWrapper::ThreadParam publish_odom_thread_param("Publish sensor data thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2, 1);
    thread_manager->RegisterThread(std::thread(&zima_ros::ZimaNode::PublishSensorDataThread, &test, publish_odom_thread_param), publish_odom_thread_param);

    // 作用：发布用于 RViz 可视化的数据，机器人状态、地图、路径等
    // 协作：从operation_data_等共享数据结构中获取最新状态供可视化
    zima::ZimaThreadWrapper::ThreadParam publish_for_rviz_param("Publish for rviz thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.3, 1);
    thread_manager->RegisterThread(std::thread(&zima_ros::ZimaNode::PublishForRvizThread, &test, publish_for_rviz_param), publish_for_rviz_param);

    // 作用：处理数据相关任务，对传感器数据、地图数据进行预处理、融合、分析等操作，为核心逻辑和可视化提供支持
    // 协作：与传感器数据发布、仿真线程共享和处理数据
    zima::ZimaThreadWrapper::ThreadParam data_process_thread_param("Data process thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2, 1);
    thread_manager->RegisterThread(std::thread(&zima_ros::ZimaNode::DataProcessThread, &test, data_process_thread_param), data_process_thread_param);

    // 作用：运行核心的 Gazebo 清扫仿真逻辑，负责机器人底盘控制、模式切换、slam地图同步、操作数据管理，相当于整个系统的大脑
    // 协作：读取和更新operation_data_、与数据处理、可视化线程共享机器人状态和地图、键盘事件响应、底盘控制
    zima::ZimaThreadWrapper::ThreadParam core_thread_param("Core thread", zima::ZimaThreadManager::kCoreThreadIndex_, 100, 0.2, 1);
    thread_manager->RegisterThread(std::thread(&zima_ros::ZimaGazeboNode::Run, &test, core_thread_param), core_thread_param);

    // Start running.
    while (ros::ok()) {
        zima::Time::SleepMSec(50);
    }

    test.Shutdown();

    // Exiting.
    keyboard_listener.Stop();
    const double wait_for_exit_timeout_s = 1;
    thread_manager->WaitForThreadExit(ros_thread_param.thread_name_, wait_for_exit_timeout_s);
    thread_manager->WaitForThreadExit(tf_thread_param.thread_name_, wait_for_exit_timeout_s);
    thread_manager->WaitForThreadExit(publish_odom_thread_param.thread_name_, wait_for_exit_timeout_s);
    thread_manager->WaitForThreadExit(data_process_thread_param.thread_name_, wait_for_exit_timeout_s);
    thread_manager->WaitForThreadExit(publish_for_rviz_param.thread_name_, wait_for_exit_timeout_s);
    thread_manager->WaitForThreadExit(core_thread_param.thread_name_, wait_for_exit_timeout_s);
    ZINFO << "Exit.";
    if (thread_manager->RunningThreadCount() > 1) {
        thread_manager->DebugRunningThreadName();
    }

    // 关闭 protobuf 库
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
