
#include <atomic>
#include <csignal>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "Timer.h"
#include "camera.h"
#include "control.h"
#include "global.h"

std::atomic<bool> running(true);

void signalHandler(int signal)
{
    running.store(false);
}

int main(void)
{
    std::signal(SIGINT, signalHandler);

    // 初始化摄像头
    double dest_fps = readDoubleFromFile(destfps_file);
    int dest_frame_duration = CameraInit(0, dest_fps, 320, 240);

    if (dest_frame_duration == -1) {
        std::cerr << "摄像头初始化失败！" << std::endl;
        return -1;
    }
    std::cout << "Camera Init Success!" << std::endl;

    // 初始化控制
    ControlInit();
    std::cout << "Control Initialized!" << std::endl;

    std::cout << "\n=== 单线程主循环开始 ===" << std::endl;
    std::cout << "按 Ctrl+C 停止程序\n" << std::endl;

    auto last_control_time = std::chrono::steady_clock::now();
    const auto control_interval = std::chrono::milliseconds(50);  // 20Hz

    while (running.load()) {
        // 1. 读取摄像头图像（单线程直接读取）
        if (!cap.read(pubframe) || pubframe.empty()) {
            std::cerr << "摄像头读取失败" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 2. 处理图像
        raw_frame = pubframe.clone();
        if (CameraHandler() != 0) {
            // 图像处理失败，继续下一帧
            continue;
        }

        // 3. 控制小车（20Hz）
        auto now = std::chrono::steady_clock::now();
        if (now - last_control_time >= control_interval) {
            // 更新参数
            target_speed = readDoubleFromFile(speed_file);
            servo_mid = static_cast<int>(readDoubleFromFile(servo_mid_file));
            if (std::abs(servo_mid) > 900) {
                std::cerr << "Warning: servoMid out of range (" << servo_mid
                          << "), reset to 0" << std::endl;
                servo_mid = 0;
            }
            kp = readDoubleFromFile(kp_file);
            kd = readDoubleFromFile(kd_file);

            // 执行控制
            ControlMain();
            last_control_time = now;
        }

        // 4. 帧率控制
        std::this_thread::sleep_for(std::chrono::milliseconds(dest_frame_duration));
    }

    std::cout << "\nStopping!" << std::endl;

    // 停止小车
    ControlExit();
    std::cout << "Control Service stopped!" << std::endl;

    // 释放摄像头
    cameraDeInit();
    std::cout << "Camera Service stopped!" << std::endl;

    return 0;
}
