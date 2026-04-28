#include "serial.h"

#include <fstream>
#include <iostream>
#include <string>

char vofa_buffer[64];

bool vofa_justfloat(int CH_count)
{
    const unsigned char tail[4] { 0x00, 0x00, 0x80, 0x7f };
    std::string path = "/dev/ttyUSB0";
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return false;
    }
    file.write((char*)vofa_buffer, CH_count * 4);
    file.write((char*)tail, sizeof(tail));
    file.close();
    return true;
}

bool vofa_image(int IMG_ID, int IMG_SIZE, int IMG_WIDTH, int IMG_HEIGHT, ImgFormat IMG_FORMAT, char* image)
{
    int preFrame[7] = { 0, 0, 0, 0, 0, 0x7F800000, 0x7F800000 };
    preFrame[0] = IMG_ID;
    preFrame[1] = IMG_SIZE;
    preFrame[2] = IMG_WIDTH;
    preFrame[3] = IMG_HEIGHT;
    preFrame[4] = IMG_FORMAT;
    std::string path = "/dev/ttyUSB0";
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return false;
    }
    file.write((char*)preFrame, sizeof(preFrame));
    file.write((char*)image, IMG_SIZE);
    file.close();
    return true;
}