
#ifndef GPIO_H
#define GPIO_H

#include <cstring>
#include <fcntl.h>
#include <string>
#include <unistd.h>

class GPIO {
public:
    GPIO(int gpioNum);
    ~GPIO();

    bool setDirection(const std::string& direction);
    bool setEdge(const std::string& edge);
    bool setValue(bool value); // 设置 GPIO 输出值
    bool readValue(); // 读取 GPIO 输入值
    int getFileDescriptor() const; // 获取 GPIO 文件描述符

private:
    int gpioNum;
    int fd; // 文件描述符
    std::string gpioPath;

    bool writeToFile(const std::string& path, const std::string& value);
};

#endif // GPIO_H
