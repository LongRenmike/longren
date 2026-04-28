
#include "global.h"

#include <fstream>
#include <iostream>

double target_speed;
int servo_mid;

// 从文件读取双精度值
double readDoubleFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    double value = 0.0;
    if (file.is_open()) {
        file >> value; // 读取文件中的值
        file.close();
    } else {
        std::cerr << "Failed to open " << filename << std::endl;
    }
    return value;
}

// 从文件中读取标志
bool readFlag(const std::string& filename)
{
    std::ifstream file(filename);
    int flag = 0;
    if (file.is_open()) {
        file >> flag; // 读取文件中的更新标志
        file.close();
    } else {
        std::cerr << "Failed to open " << filename << std::endl;
    }
    return flag;
}
