
#include "image_cv.h"
#include <iostream>
#include "global.h"
#include <vector>
#include <deque>
#include <cmath>

#include <fcntl.h>
#include <linux/fb.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <numeric>

cv::Mat raw_frame;
cv::Mat grayFrame;
cv::Mat binarizedFrame;
cv::Mat morphologyExFrame;
cv::Mat track;

std::vector<int> left_line;
std::vector<int> right_line;
std::vector<int> mid_line;

int line_tracking_width;
int line_tracking_height;

bool zebraLineDetected = false;

int rectangleCount = 0;

int yellowAreaCount;
int whiteAreaCount;

bool detectRedBlock(const cv::Mat& inputImage)
{
    if (inputImage.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return false;
    }

    cv::Mat hsvImage;
    cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Scalar lowerRed1(0, 100, 100);
    cv::Scalar upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(160, 100, 100);
    cv::Scalar upperRed2(180, 255, 255);

    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(hsvImage, lowerRed1, upperRed1, redMask1);
    cv::inRange(hsvImage, lowerRed2, upperRed2, redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(redMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int minContourArea = 5;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > minContourArea) {
            std::cout << "Red block detected, area: " << area << std::endl;
            return true;
        }
    }

    return false;
}

int detectYellowBlocks(const cv::Mat& inputImage)
{
    if (inputImage.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return -1;
    }

    cv::Mat hsvImage;
    cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Scalar lowerYellow(10, 100, 100);
    cv::Scalar upperYellow(40, 255, 255);

    cv::Mat yellowMask;
    inRange(hsvImage, lowerYellow, upperYellow, yellowMask);

    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(yellowMask, yellowMask, cv::MORPH_OPEN, kernel);
    morphologyEx(yellowMask, yellowMask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(yellowMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int minContourArea = 4;
    int yellowBlockCount = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > minContourArea) {
            yellowBlockCount++;
        }
    }

    return yellowBlockCount;
}

int detectWhiteBlocks(const cv::Mat& inputImage)
{
    if (inputImage.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return -1;
    }

    cv::Mat hsvImage;
    cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Scalar lowerWhite(108, 0, 160);
    cv::Scalar upperWhite(129, 12, 203);

    cv::Mat whiteMask;
    inRange(hsvImage, lowerWhite, upperWhite, whiteMask);

    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(10, 5));
    morphologyEx(whiteMask, whiteMask, cv::MORPH_OPEN, kernel);
    morphologyEx(whiteMask, whiteMask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(whiteMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int minContourArea = 400;
    int whiteBlockCount = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > minContourArea) {
            whiteBlockCount++;
        }
    }

    if (whiteBlockCount >= 5)
        return 1;
    else
        return 0;
}

double calculateOtsuThresholdForRegion(const cv::Mat& grayImage, int startRow, int endRow, int startCol, int endCol)
{
    if (grayImage.empty()) {
        throw std::invalid_argument("Input image is empty!");
    }

    if (grayImage.type() != CV_8UC1) {
        throw std::invalid_argument("Input image must be a grayscale image (CV_8UC1)!");
    }

    int height = grayImage.rows;
    int width = grayImage.cols;
    if (startRow < 0 || endRow >= height || startRow > endRow ||
        startCol < 0 || endCol >= width || startCol > endCol) {
        throw std::invalid_argument("Invalid row or column range specified!");
    }

    int hist[256] = {0};
    int pixelCount = 0;

    for (int y = startRow; y <= endRow; y++) {
        const uchar* rowPtr = grayImage.ptr<uchar>(y);
        for (int x = startCol; x <= endCol; x++) {
            hist[rowPtr[x]]++;
            pixelCount++;
        }
    }

    if (pixelCount == 0) {
        throw std::runtime_error("Selected region contains no pixels.");
    }

    double total = pixelCount;
    double sum = 0.0;
    for (int i = 0; i < 256; i++) {
        sum += i * hist[i];
    }

    double sumB = 0.0;
    double wB = 0.0;
    double varMax = 0.0;
    double threshold = 0.0;

    for (int t = 0; t < 256; t++) {
        wB += hist[t];
        if (wB == 0) continue;

        double wF = total - wB;
        if (wF == 0) break;

        sumB += t * hist[t];

        double mB = sumB / wB;
        double mF = (sum - sumB) / wF;

        double varBetween = wB * wF * (mB - mF) * (mB - mF);

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = t;
        }
    }

    return threshold;
}

bool detectZebraCrossing(const cv::Mat& gray)
{
    const int height = gray.rows;
    const int width = gray.cols;

    const int scan_y_start = height * 0.20;
    const int scan_y_end = height * 0.60;
    const int x_start = width * 0.2;
    const int x_end = width * 0.8;

    int GRAY_THRESHOLD = calculateOtsuThresholdForRegion(
        gray,
        scan_y_start,
        scan_y_end,
        x_start,
        x_end
    );

    const int LINE_SPACING = 2;
    const int scan_lines = (scan_y_end - scan_y_start) / LINE_SPACING;
    const int MIN_TRANSITIONS = 3;

    int total_transitions = 0;

    for (int i = 0; i < scan_lines; ++i) {
        int current_y = scan_y_start + i * LINE_SPACING;
        bool current_white = gray.at<uchar>(current_y, x_start) > GRAY_THRESHOLD;
        int line_transitions = 0;

        uchar prev_val = gray.at<uchar>(current_y, x_start);

        for (int x = x_start + 1; x < x_end; x++) {
            uchar curr_val = gray.at<uchar>(current_y, x);

            if (abs(curr_val - prev_val) < 20) {
                prev_val = curr_val;
                continue;
            }

            bool pixel_white = curr_val > GRAY_THRESHOLD;
            if (pixel_white != current_white) {
                line_transitions++;
                current_white = pixel_white;
            }
            prev_val = curr_val;
        }

        total_transitions += line_transitions;
    }

    double average_transitions = static_cast<double>(total_transitions) / scan_lines;

    return (average_transitions >= MIN_TRANSITIONS);
}

cv::Mat image_binerize(cv::Mat& frame)
{
    cv::Mat output;
    cv::Mat binarizedFrame;
    cv::Mat hsvImage;
    cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);

    cv::threshold(hsvChannels[0], binarizedFrame, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::threshold(hsvChannels[1], output, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    cv::bitwise_or(output, binarizedFrame, output);

    return output;
}

int W = 0;
int H = 0;

int Left_Lost_Time = 0;
int Right_Lost_Time = 0;
int Both_Lost_Time = 0;
int Boundry_Start_Left = 0;
int Boundry_Start_Right = 0;

std::vector<int> Road_Wide;
std::vector<int> Left_Lost_Flag;
std::vector<int> Right_Lost_Flag;

int Left_Up_Find = 0;
int Right_Up_Find = 0;

int Left_Down_Find = 0;
int Right_Down_Find = 0;

int Left_Down_NB_Find = 0;
int Right_Down_NB_Find = 0;

int start = 0;
int end = 0;

int stop_row = 0;

void findTrackByLongestWhiteColumn(const cv::Mat& binaryImg,
                                    std::vector<int>& left_line,
                                    std::vector<int>& right_line,
                                    std::vector<int>& mid_line)
{
    CV_Assert(!binaryImg.empty() && binaryImg.type() == CV_8UC1);

    const int start_column = 20;
    const int end_column = W - 20;

    std::vector<int> white_column(W, 0);

    for (int j = start_column; j <= end_column; ++j) {
        for (int i = H - 1; i >= 0; --i) {
            if (binaryImg.at<uchar>(i, j) == 0) break;
            white_column[j]++;
        }
    }

    int max_left_len = 0, max_left_col = start_column;
    for (int j = start_column; j <= end_column; ++j) {
        if (white_column[j] > max_left_len) {
            max_left_len = white_column[j];
            max_left_col = j;
        }
    }
    int max_right_len = 0, max_right_col = end_column;
    for (int j = end_column; j >= start_column; j--) {
        if (white_column[j] > max_right_len) {
            max_right_len = white_column[j];
            max_right_col = j;
        }
    }

    stop_row = std::min(H - 1, H - max_left_len);

    left_line.assign(H, -1);
    right_line.assign(H, -1);
    mid_line.assign(H, -1);

    Left_Lost_Time = Right_Lost_Time = Both_Lost_Time = 0;
    Boundry_Start_Left = Boundry_Start_Right = 0;

    for (int i = H - 1; i >= stop_row; --i) {
        int right_border = W - 1;
        for (int j = max_right_col; j < W - 2; ++j) {
            if (binaryImg.at<uchar>(i, j) == 255 &&
                binaryImg.at<uchar>(i, j + 1) == 0 &&
                binaryImg.at<uchar>(i, j + 2) == 0) {
                right_border = j;
                break;
            }
        }

        int left_border = 0;
        for (int j = max_left_col; j >= 2; --j) {
            if (binaryImg.at<uchar>(i, j) == 255 &&
                binaryImg.at<uchar>(i, j - 1) == 0 &&
                binaryImg.at<uchar>(i, j - 2) == 0) {
                left_border = j;
                break;
            }
        }

        left_line[i] = left_border + 10;
        right_line[i] = right_border - 10;
        mid_line[i] = (left_line[i] + right_line[i]) / 2;

        Left_Lost_Flag[i] = (left_border <= 1) ? 1 : 0;
        Right_Lost_Flag[i] = (right_border >= W - 2) ? 1 : 0;

        if (i > stop_row && i < 35) {
            if (Left_Lost_Flag[i]) Left_Lost_Time++;
            if (Right_Lost_Flag[i]) Right_Lost_Time++;
            if (Left_Lost_Flag[i] && Right_Lost_Flag[i]) Both_Lost_Time++;
        }

        if (Boundry_Start_Left == 0 && Left_Lost_Flag[i] == 0)
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] == 0)
            Boundry_Start_Right = i;

        Road_Wide[i] = right_border - left_border;
    }

    for (int i = stop_row - 1; i >= 0; --i) {
        left_line[i] = left_line[i + 5];
        right_line[i] = right_line[i + 5];
        mid_line[i] = (left_line[i] + right_line[i]) / 2;
        Left_Lost_Flag[i] = Left_Lost_Flag[i + 1];
        Right_Lost_Flag[i] = Right_Lost_Flag[i + 1];
        Road_Wide[i] = Road_Wide[i + 1];
    }
    return;
}

void image_main()
{
    cv::Mat resizedFrame;

    cv::resize(raw_frame, resizedFrame, cv::Size(line_tracking_width, line_tracking_height));

    cv::cvtColor(resizedFrame, grayFrame, cv::COLOR_BGR2GRAY);

    if (detectZebraCrossing(grayFrame)) {
        zebraLineDetected = true;
    } else {
        zebraLineDetected = false;
    }

    binarizedFrame = image_binerize(resizedFrame);

    W = line_tracking_width;
    H = line_tracking_height;

    left_line.clear();
    right_line.clear();
    mid_line.clear();

    left_line.resize(line_tracking_height, -1);
    right_line.resize(line_tracking_height, -1);
    mid_line.resize(line_tracking_height, -1);
    Road_Wide.resize(line_tracking_height);
    Left_Lost_Flag.resize(line_tracking_height);
    Right_Lost_Flag.resize(line_tracking_height);

    findTrackByLongestWhiteColumn(binarizedFrame, left_line, right_line, mid_line);
}
