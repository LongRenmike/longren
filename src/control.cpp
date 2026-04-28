
#include "control.h"

#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>

#include "global.h"
#include "traffic_stop.h"

extern double angle;
extern std::atomic<bool> zebra_signal;
extern std::atomic<bool> traffic_stop_signal;

static int serial_fd = -1;
static constexpr const char* kControlSerialDevice = "/dev/ttyUSB0";
static constexpr int kControlSerialBaudrate = 230400;
static constexpr int kDiffAssistCommandId = 0x51;

int serialInit(const char* device, int baudrate)
{
    serial_fd = open(device, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port: " << device << " Error: " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return -1;
    }

    speed_t baud;
    switch (baudrate) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        default:
            // We require 230400 for the car control link; never silently fall back to 115200.
            std::cerr << "Warning: Unsupported baudrate " << baudrate
                      << ", forcing " << kControlSerialBaudrate << std::endl;
            baud = B230400;
            baudrate = kControlSerialBaudrate;
            break;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_fd);
        return -1;
    }

    std::cout << "Serial port initialized: " << device << " at " << baudrate << " baud" << std::endl;
    return 0;
}

void serialClose()
{
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}

// 发送速度包 (阿克曼协议)
// 格式: [0xAA, 0x55, 0x0B, 0x50] + speed_cmd(2字节大端有符号) + 0(2字节) + 0(2字节) + checksum(1字节)
int sendSpeedPacket(int speed_cmd)
{
    if (serial_fd < 0) {
        return -1;
    }

    uint8_t packet[11];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 0x0B;  // 长度11
    packet[3] = 0x50;  // 速度命令

    // speed_cmd: 大端有符号16位
    packet[4] = (speed_cmd >> 8) & 0xFF;
    packet[5] = speed_cmd & 0xFF;

    // 填充0
    packet[6] = 0x00;
    packet[7] = 0x00;
    packet[8] = 0x00;
    packet[9] = 0x00;

    // 校验和
    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) {
        checksum += packet[i];
    }
    packet[10] = checksum;

    // 重试机制
    for (int retry = 0; retry < 3; retry++) {
        ssize_t written = write(serial_fd, packet, 11);
        if (written == 11) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cerr << "Failed to send speed packet after 3 retries" << std::endl;
    return -1;
}

// 发送左右轮差速速度包
// 格式: [0xAA, 0x55, 0x0B, 0x51] + left_cmd(2字节大端有符号) + right_cmd(2字节大端有符号) + 0(2字节) + checksum(1字节)
int sendDifferentialSpeedPacket(int left_cmd, int right_cmd)
{
    if (serial_fd < 0) {
        return -1;
    }

    uint8_t packet[11];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 0x0B;
    packet[3] = kDiffAssistCommandId;

    packet[4] = (left_cmd >> 8) & 0xFF;
    packet[5] = left_cmd & 0xFF;
    packet[6] = (right_cmd >> 8) & 0xFF;
    packet[7] = right_cmd & 0xFF;
    packet[8] = 0x00;
    packet[9] = 0x00;

    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) {
        checksum += packet[i];
    }
    packet[10] = checksum;

    for (int retry = 0; retry < 3; retry++) {
        ssize_t written = write(serial_fd, packet, 11);
        if (written == 11) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cerr << "Failed to send differential speed packet after 3 retries" << std::endl;
    return -1;
}

// 发送舵机包 (阿克曼协议)
// 格式: [0xAA, 0x55, 0x11, 0x60] + steer_cmd(2字节大端有符号) + 0(10字节) + checksum(1字节)
int sendServoPacket(int steer_cmd)
{
    if (serial_fd < 0) {
        return -1;
    }

    uint8_t packet[17];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 0x11;  // 长度17
    packet[3] = 0x60;  // 舵机命令

    // steer_cmd: 大端有符号16位，单位0.1度
    packet[4] = (steer_cmd >> 8) & 0xFF;
    packet[5] = steer_cmd & 0xFF;

    // 填充0 (10字节)
    for (int i = 6; i < 16; i++) {
        packet[i] = 0x00;
    }

    // 校验和
    uint8_t checksum = 0;
    for (int i = 0; i < 16; i++) {
        checksum += packet[i];
    }
    packet[16] = checksum;

    // 重试机制
    for (int retry = 0; retry < 3; retry++) {
        ssize_t written = write(serial_fd, packet, 17);
        if (written == 17) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cerr << "Failed to send servo packet after 3 retries" << std::endl;
    return -1;
}

// 将角度转换为舵机命令值 (0.1度单位)
// angle: 弧度，范围约 -1.57 到 1.57 (-90度到90度)
int angleToSteerCmd(double angle_rad)
{
    // 转换为度
    double angle_deg = angle_rad * 180.0 / M_PI;
    // 转换为0.1度单位
    int steer_cmd = static_cast<int>(angle_deg * 10.0);
    // 限制范围
    if (steer_cmd > 900) steer_cmd = 900;    // 90度
    if (steer_cmd < -900) steer_cmd = -900;  // -90度
    return steer_cmd;
}

// 速度限制 (m/s 转 mm/s)
int speedToCmd(double speed_mps)
{
    int speed_cmd = static_cast<int>(speed_mps * 1000.0);  // m/s 转 mm/s
    // 限制范围
    if (speed_cmd > 1200) speed_cmd = 1200;
    if (speed_cmd < -1200) speed_cmd = -1200;
    return speed_cmd;
}

int clampSpeedCmd(int speed_cmd)
{
    if (speed_cmd > 1200) return 1200;
    if (speed_cmd < -1200) return -1200;
    return speed_cmd;
}

void ControlInit()
{
    if (serialInit(kControlSerialDevice, kControlSerialBaudrate) < 0) {
        std::cerr << "Serial init failed!" << std::endl;
    }
}

void ControlMain()
{
    double target_speed_mps = 0.0;
    double current_angle = 0.0;

    int start_val = readFlag(start_file) ? 1 : 0;
    int zebra_val = zebra_signal ? 1 : 0;
    int traffic_stop_val = traffic_stop_signal ? 1 : 0;

    if (!readFlag(start_file)) {
        target_speed_mps = 0.0;
        current_angle = 0.0;
    } else if (zebra_signal || traffic_stop_signal) {
        // 斑马线礼让：强制停车（持续发送 0 速度，直到 zebra_signal 变为 false）
        target_speed_mps = 0.0;
        current_angle = angle;
    } else {
        target_speed_mps = target_speed;
        current_angle = angle;
    }

    // 转换速度
    int speed_cmd = speedToCmd(target_speed_mps);

    // 转换角度为舵机命令
    int raw_steer_cmd = angleToSteerCmd(current_angle);
    int steer_cmd = raw_steer_cmd + servo_mid;
    if (steer_cmd > 900) steer_cmd = 900;
    if (steer_cmd < -900) steer_cmd = -900;

    const bool diff_enable = readFlag(diff_enable_file);
    const int diff_threshold = static_cast<int>(readDoubleFromFile(diff_threshold_file));
    const double diff_gain = readDoubleFromFile(diff_gain_file);
    const int diff_limit = static_cast<int>(readDoubleFromFile(diff_limit_file));
    const int diff_sign = (readDoubleFromFile(diff_sign_file) >= 0.0) ? 1 : -1;

    bool diff_active = false;
    int left_speed_cmd = speed_cmd;
    int right_speed_cmd = speed_cmd;

    if (diff_enable && std::abs(steer_cmd) > diff_threshold && speed_cmd != 0) {
        const int steer_over_threshold = std::abs(steer_cmd) - diff_threshold;
        const int file_limited_assist = std::max(0, static_cast<int>(steer_over_threshold * diff_gain));
        const int speed_limited_assist = std::max(0, static_cast<int>(std::abs(speed_cmd) * 0.4));
        const int assist_cmd = std::min(diff_limit, std::min(file_limited_assist, speed_limited_assist));

        if (assist_cmd > 0) {
            // steer_cmd > 0 时默认右轮更快，帮助车尾跟上过弯。
            const int signed_assist = (steer_cmd > 0 ? 1 : -1) * diff_sign * assist_cmd;
            left_speed_cmd = clampSpeedCmd(speed_cmd - signed_assist);
            right_speed_cmd = clampSpeedCmd(speed_cmd + signed_assist);
            diff_active = true;
        }
    }

    // 调试输出
    static int print_count = 0;
    if (print_count % 20 == 0) {
        std::cout << "Control: start=" << start_val 
                  << ", zebra=" << zebra_val 
                  << ", traffic_stop=" << traffic_stop_val
                  << ", speed_mps=" << target_speed_mps 
                  << ", speed_cmd=" << speed_cmd 
                  << ", angle=" << current_angle 
                  << ", raw_steer_cmd=" << raw_steer_cmd
                  << ", servo_mid=" << servo_mid
                  << ", steer_cmd=" << steer_cmd
                  << ", diff_active=" << (diff_active ? 1 : 0)
                  << ", left_speed_cmd=" << left_speed_cmd
                  << ", right_speed_cmd=" << right_speed_cmd << std::endl;
    }
    print_count++;

    if (diff_active) {
        sendDifferentialSpeedPacket(left_speed_cmd, right_speed_cmd);
    } else {
        sendSpeedPacket(speed_cmd);
    }

    // 发送舵机包
    sendServoPacket(steer_cmd);
}

void ControlExit()
{
    // 停止小车（先检查串口是否有效）
    if (serial_fd >= 0) {
        sendSpeedPacket(0);
        sendDifferentialSpeedPacket(0, 0);
        sendServoPacket(0);
    }
    serialClose();
    std::cout << "Control Service stopped!" << std::endl;
}
