
#include "camera.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <chrono>

#include "PIDController.h"
#include "frame_buffer.h"
#include "global.h"
#include "traffic_stop.h"

cv::VideoCapture cap;

double kp = 0;
double ki = 0;
double kd = 0;
double error_kp = 0;
double error_ki = 0;
double error_kd = 0;
double errorduty = 0;
double angle = 0;

int screenWidth, screenHeight;
int newWidth, newHeight;
int fb;
uint16_t* fb_buffer;

bool zebra_mark = 0;
static int zebra_detect_streak = 0;
constexpr int ZEBRA_CONFIRM_FRAMES = 4;

static auto lastZebraTime = std::chrono::steady_clock::now();
constexpr auto DEBOUNCE_INTERVAL = std::chrono::milliseconds(500);

std::atomic<bool> zebra_signal(false);
std::atomic<bool> thread_running(false);

#define calc_scale 2

const double MAX_DELTA = 30.0;
static double prev_show = 0.0;

double filterDelta(double current_show)
{
    double delta = current_show - prev_show;
    if (std::abs(delta) > MAX_DELTA) {
        current_show = prev_show;
    }
    prev_show = current_show;
    return current_show;
}

// 输出上限按 ±90° 对齐，避免转向被过早限死在很小角度
PIDController ServoControl(0.0, 0.0, 0.0, 0.0, POSITION, 100.0);

double camera_error = 0.0;
const int Weight[60] =
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 6, 7, 9, 11, 13, 15, 17, 19,
    25, 25, 19, 17, 15, 13, 11, 9, 7, 6,
    1, 1, 1, 1, 1, 1, 1, 1, 1,
};

int CameraInit(uint8_t camera_id, double dest_fps, int width, int height)
{
    // 阿克曼式小车通过串口控制转向，不使用这里的硬件PWM舵机。

    fb = open("/dev/fb0", O_RDWR);
    if (fb == -1) {
        std::cerr << "无法打开帧缓冲区设备" << std::endl;
        return -1;
    }

    struct fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        std::cerr << "无法获取帧缓冲区信息" << std::endl;
        close(fb);
        return -1;
    }

    screenWidth = vinfo.xres;
    screenHeight = vinfo.yres;

    size_t fb_size = vinfo.yres_virtual * vinfo.xres_virtual * vinfo.bits_per_pixel / 8;

    fb_buffer = (uint16_t*)mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    if (fb_buffer == MAP_FAILED) {
        std::cerr << "无法映射帧缓冲区到内存" << std::endl;
        close(fb);
        return -1;
    }

    cap.open(0);

    if (!cap.isOpened()) {
        printf("无法打开摄像头\n");
        munmap(fb_buffer, fb_size);
        close(fb);
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, -1);

    int cameraWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int cameraHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("摄像头分辨率: %d x %d\n", cameraWidth, cameraHeight);

    cv::Mat testFrame;
    for (int i = 0; i < 10; i++) {
        cap.read(testFrame);
        if (!testFrame.empty()) {
            printf("摄像头测试读取成功: %d x %d\n", testFrame.cols, testFrame.rows);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    double widthRatio = static_cast<double>(screenWidth) / cameraWidth;
    double heightRatio = static_cast<double>(screenHeight) / cameraHeight;
    double scale = std::min(widthRatio, heightRatio);

    newWidth = static_cast<int>(cameraWidth * scale);
    newHeight = static_cast<int>(cameraHeight * scale);
    printf("自适应分辨率: %d x %d\n", newWidth, newHeight);

    double fps = cap.get(cv::CAP_PROP_FPS);
    printf("Camera fps:%lf\n", fps);

    line_tracking_width = newWidth / calc_scale;
    line_tracking_height = newHeight / calc_scale;

    return static_cast<int>(1000.0 / std::min(fps, dest_fps));
}

void cameraDeInit(void)
{
    cap.release();

    struct fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        std::cerr << "无法获取帧缓冲区信息" << std::endl;
    } else {
        size_t fb_size = vinfo.yres_virtual * vinfo.xres_virtual * vinfo.bits_per_pixel / 8;
        munmap(fb_buffer, fb_size);
    }

    close(fb);
}

int saved_frame_count = 0;
bool saveCameraImage(cv::Mat frame, const std::string& directory)
{
    if (frame.empty()) {
        std::cerr << "Save Error: Frame is empty." << std::endl;
        return 0;
    }

    std::ostringstream filename;
    filename << directory << "/image_" << std::setw(5) << std::setfill('0') << saved_frame_count << ".jpg";

    saved_frame_count++;
    return cv::imwrite(filename.str(), frame);
}

// 单线程版本：不需要 streamCapture 线程
// 图像直接在 main.cpp 中读取
cv::Mat pubframe;

void signal_thread_func()
{
    zebra_signal = true;
    std::cout << "人行道前停车礼让行人！" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    zebra_signal = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    thread_running = false;
}

int CameraHandler(void)
{
    cv::Mat resizedFrame;

    // 单线程版本：直接使用 raw_frame（已在 main.cpp 中赋值）
    if (raw_frame.empty()) {
        printf("图像为空\n");
        return -1;
    }

    if (readFlag(saveImg_file)) {
        if (saveCameraImage(raw_frame, "./image")) {
            printf("图像%d已保存\n", saved_frame_count);
        } else {
            printf("图像保存失败\n");
            return -1;
        }
    }

    {
        image_main();
        updateTrafficStopState(raw_frame);

        if (zebraLineDetected) {
            zebra_detect_streak++;
            if (zebra_detect_streak >= ZEBRA_CONFIRM_FRAMES && !zebra_mark && !thread_running) {
                auto currentZebraTime = std::chrono::steady_clock::now();

                if (currentZebraTime - lastZebraTime >= DEBOUNCE_INTERVAL) {
                    zebra_mark = 1;
                    lastZebraTime = currentZebraTime;
                    thread_running = true;
                    std::thread(signal_thread_func).detach();
                }
            }
        } else {
            zebra_detect_streak = 0;
            zebra_mark = 0;
        }
    }

    if (readFlag(start_file)) {
        int foresee = readDoubleFromFile(foresee_file);

        if (mid_line[foresee] != 255) {
            float Weight_count = 0;
            double camera_error_sum = 0;
            for (int i = 59; i >= 15; i--) {
                camera_error_sum += (mid_line[(i) / calc_scale] * calc_scale - newWidth / 2.0) * Weight[i];
                Weight_count += Weight[i];
            }
            camera_error = camera_error_sum / Weight_count;

            // 计算 PID 输出角度（弧度）
            ServoControl.setPID(kp, 0, kd);
            double servoduty = -ServoControl.update(camera_error);
            
            // 将 PID 输出转换为舵机角度（弧度）
            // servoduty 范围大约是 -100 到 100，映射到 -1.57 到 1.57 弧度
            angle = servoduty / 100.0 * 1.57;
            
            // 限制角度范围
            if (angle > 1.57) angle = 1.57;
            if (angle < -1.57) angle = -1.57;

            static int angle_print_count = 0;
            if (angle_print_count % 20 == 0) {
                std::cout << "Camera: camera_error=" << camera_error
                          << ", servoduty=" << servoduty
                          << ", angle_rad=" << angle
                          << ", angle_deg=" << (angle * 180.0 / M_PI)
                          << std::endl;
            }
            angle_print_count++;
        }
    }

    if (readFlag(showImg_file)) {
        cv::Mat fbImage(screenHeight, screenWidth, CV_8UC3, cv::Scalar(0, 0, 0));

        cv::resize(binarizedFrame, resizedFrame, cv::Size(newWidth, newHeight));
        cv::Mat coloredResizedFrame;
        cv::cvtColor(resizedFrame, coloredResizedFrame, cv::COLOR_GRAY2BGR);

        fbImage.setTo(cv::Scalar(0, 0, 0));
        cv::Rect roi((screenWidth - newWidth) / 2, (screenHeight - newHeight) / 2, newWidth, newHeight);
        coloredResizedFrame.copyTo(fbImage(roi));

        if (traffic_stop_signal) {
            cv::putText(fbImage, "TRAFFIC STOP", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        }

        int scaledLeftX, scaledRightX, scaledMidX, scaledY;

        for (int y = 0; y < line_tracking_height; y++) {
            scaledLeftX = static_cast<int>(left_line[y] * calc_scale);
            scaledRightX = static_cast<int>(right_line[y] * calc_scale);
            scaledMidX = static_cast<int>(mid_line[y] * calc_scale);
            scaledY = static_cast<int>(y * calc_scale);

            cv::line(fbImage(roi), cv::Point(scaledLeftX, scaledY), cv::Point(scaledLeftX, scaledY), cv::Scalar(0, 0, 255), calc_scale);
            cv::line(fbImage(roi), cv::Point(scaledRightX, scaledY), cv::Point(scaledRightX, scaledY), cv::Scalar(0, 255, 0), calc_scale);
            cv::line(fbImage(roi), cv::Point(scaledMidX, scaledY), cv::Point(scaledMidX, scaledY), cv::Scalar(255, 0, 0), calc_scale);
        }

        convertMatToRGB565(fbImage, fb_buffer, screenWidth, screenHeight);
    }
    return 0;
}
