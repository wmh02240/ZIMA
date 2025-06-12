/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/system/keyboard_listener.h"

#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

KeyboardListener::KeyboardListener(const CallBackFunc &cb_func) {
    tcgetattr(STDIN_FILENO, &init_settings_);
    auto new_settings = init_settings_;

    // No need to press enter. 关闭规范模式（不需要回车即可读取输入）
    new_settings.c_lflag &= (~ICANON);

    // Enable terminal display.
    // new_settings.c_lflag |= ECHO;

    // Disable terminal display. 关闭回显（输入时不在终端显示）
    new_settings.c_lflag &= (~ECHO);

    // 保持信号处理
    new_settings.c_lflag |= ISIG;

    // 设置读取参数：立即返回、至少读1个字符
    new_settings.c_cc[VTIME] = 0;
    new_settings.c_cc[VMIN] = 1;

    // 应用新设置
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    // 线程参数初始化
    thread_param_ = ZimaThreadWrapper::ThreadParam("Keyboard listener thread", ZimaThreadManager::kMiscThreadIndex_, 100, -1, -1);
    cb_func_ = cb_func; // 回调函数 cb_func_ 用于处理每次按键事件。
    working_.store(false);
    ZWARN << "Please do not remove callback function before stopping this listener.";
}

KeyboardListener::~KeyboardListener() {
    Stop();
    ZINFO << "Thread exit.";
    tcsetattr(0, TCSANOW, &init_settings_); // 析构时恢复，避免影响shell
}

// 检查是否已在运行，未运行则启动监听线程
bool KeyboardListener::Start() {
    if (working_.load()) {
        ZINFO << "Thread already working.";
        return true;
    }
    auto thread_manager = ZimaThreadManager::Instance();
    working_.store(true);
    if (!thread_manager->IsThreadRunning(thread_param_.thread_name_)) {
        thread_manager->RegisterThread(std::thread(&KeyboardListener::OperateThread, this, thread_param_), thread_param_);
    }
    return true;
}

// 设置 working_ 为 false，通知线程退出，并等待线程安全结束
bool KeyboardListener::Stop() {
    if (!working_.load()) {
        ZINFO << "Thread already stopped.";
        return true;
    }
    working_.store(false);
    // Make sure thread already exit.
    auto thread_manager = ZimaThreadManager::Instance();
    thread_manager->WaitForThreadExit(thread_param_.thread_name_, 1);
    return true;
}

void KeyboardListener::OperateThread(const ZimaThreadWrapper::ThreadParam &param) {
    ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
    auto thread_manager = ZimaThreadManager::Instance();
    if (param.bind_cpu_id_ >= 0) {
        thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
    }

    // Method 2.
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    read_timeout.tv_usec = 0;

    fd_set fd_set_read;

    while (working_.load()) {
        thread_manager->UpdateThreadCycle(param.thread_name_);

        // Method 1, it will block.
        // auto key = getchar();
        // cb_func_(key);

        // Method 2, it won't block. 非阻塞方式监听键盘输入
        FD_ZERO(&fd_set_read);
        FD_SET(STDIN_FILENO, &fd_set_read);
        read_timeout.tv_sec = 1;
        read_timeout.tv_usec = 0;
        // 使用 select 实现非阻塞监听，每秒检测一次是否有输入
        auto fs_sel = select(STDIN_FILENO + 1, &fd_set_read, NULL, NULL, &read_timeout);
        if (fs_sel) {
            char key = fgetc(stdin);
            cb_func_(key); // 调用回调处理按键
        }
    }
    working_.store(false);

    ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
    thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__, __LINE__);
    ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

} // namespace zima
