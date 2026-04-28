#include "traffic_stop.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

bool redLightDetected = false;
std::atomic<bool> traffic_stop_signal(false);

namespace {
constexpr int kConfirmFrames = 2;
constexpr int kClearFrames = 3;
constexpr auto kStopDuration = std::chrono::seconds(3);

bool isValidRedBlob(const std::vector<cv::Point>& contour, int roiArea)
{
    const double area = cv::contourArea(contour);
    if (area < 20.0 || area > roiArea * 0.35) {
        return false;
    }

    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 0.0) {
        return false;
    }

    const cv::Rect rect = cv::boundingRect(contour);
    if (rect.width < 4 || rect.height < 4) {
        return false;
    }

    const double ratio = static_cast<double>(rect.width) / rect.height;
    if (ratio < 0.55 || ratio > 1.8) {
        return false;
    }

    const double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
    if (circularity < 0.35) {
        return false;
    }

    const double fillRatio = area / static_cast<double>(rect.area());
    return fillRatio > 0.25;
}
}  // namespace

bool detectRedLightCircle(const cv::Mat& colorImage)
{
    if (colorImage.empty() || colorImage.type() != CV_8UC3) {
        return false;
    }

    const int width = colorImage.cols;
    const int height = colorImage.rows;

    const cv::Rect roiRect(0, 0, width, height / 2);
    cv::Mat roi = colorImage(roiRect).clone();

    cv::Mat hsvImage;
    cv::cvtColor(roi, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(hsvImage, cv::Scalar(0, 70, 60), cv::Scalar(20, 255, 255), redMask1);
    cv::inRange(hsvImage, cv::Scalar(150, 70, 60), cv::Scalar(180, 255, 255), redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int validCount = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        if (isValidRedBlob(contours[i], roiRect.area())) {
            validCount++;
        }
    }

    if (validCount > 0) {
        std::cout << "Red light candidate count: " << validCount << std::endl;
        return true;
    }

    return false;
}

void updateTrafficStopState(const cv::Mat& colorImage)
{
    static int detectStreak = 0;
    static int clearStreak = 0;
    static bool waitForClear = false;
    static std::chrono::steady_clock::time_point stopUntil = std::chrono::steady_clock::now();

    const bool detected = detectRedLightCircle(colorImage);
    redLightDetected = detected;

    if (detected) {
        detectStreak++;
        clearStreak = 0;
    } else {
        detectStreak = 0;
        clearStreak++;
    }

    const auto now = std::chrono::steady_clock::now();
    if (traffic_stop_signal && now >= stopUntil) {
        traffic_stop_signal = false;
        std::cout << "Traffic stop released, resume line tracking." << std::endl;
    }

    if (waitForClear && clearStreak >= kClearFrames) {
        waitForClear = false;
    }

    if (!traffic_stop_signal && !waitForClear && detectStreak >= kConfirmFrames) {
        traffic_stop_signal = true;
        waitForClear = true;
        stopUntil = now + kStopDuration;
        std::cout << "Red light detected, stop for 3 seconds." << std::endl;
    }
}
