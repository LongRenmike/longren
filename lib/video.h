
#ifndef VIDEO_H_
#define VIDEO_H_

#include <fcntl.h>
#include <linux/fb.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include "Timer.h"

class Video {
public:
    Video(const std::string& filename, double fps);
    ~Video(void);

    Timer timer;
    cv::Mat frame;
    std::mutex frameMutex;

private:
    cv::VideoCapture cap;
    cv::Mat fbImage;
    cv::Mat resizedFrame;
    void streamCapture(void);
    int screenHeight, screenWidth;
    int newWidth, newHeight;
    int fb;
    uint16_t* fb_buffer;
};

#endif