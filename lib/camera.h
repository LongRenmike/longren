/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 08:28:56
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:03:25
 * @FilePath: /smartcar/lib/camera.h
 * @Description: 智能驾驶摄像头头文件
 */
#ifndef CAMERA_H_
#define CAMERA_H_

#include <fcntl.h>
#include <linux/fb.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <atomic>

#include "image_cv.h"
#include "traffic_stop.h"

int CameraInit(uint8_t camera_id, double dest_fps, int width, int height);
int CameraHandler(void);
void cameraDeInit(void);

// 单线程版本：不需要 streamCapture 线程
// void streamCapture(void);
// extern bool streamCaptureRunning;

// 单线程版本：图像直接在主循环中读取
extern cv::VideoCapture cap;
extern cv::Mat pubframe;

extern double kp;
extern double ki;
extern double kd;
extern double angle;

extern std::atomic<bool> zebra_signal;
extern std::atomic<bool> traffic_stop_signal;

#endif
