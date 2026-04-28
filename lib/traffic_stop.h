#ifndef TRAFFIC_STOP_H
#define TRAFFIC_STOP_H

#include <atomic>
#include <opencv2/opencv.hpp>

extern bool redLightDetected;
extern std::atomic<bool> traffic_stop_signal;

bool detectRedLightCircle(const cv::Mat& colorImage);
void updateTrafficStopState(const cv::Mat& colorImage);

#endif
