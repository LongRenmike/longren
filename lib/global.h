
#ifndef GLOBAL_H
#define GLOBAL_H

#include <atomic>
#include <string>

const std::string param_dir = "./params/";

const std::string kp_file = param_dir + "kp";
const std::string ki_file = param_dir + "ki";
const std::string kd_file = param_dir + "kd";

const std::string mortor_kp_file = param_dir + "mortor_kp";
const std::string mortor_ki_file = param_dir + "mortor_ki";
const std::string mortor_kd_file = param_dir + "mortor_kd";

const std::string start_file = param_dir + "start";
const std::string showImg_file = param_dir + "showImg";
const std::string destfps_file = param_dir + "destfps";
const std::string foresee_file = param_dir + "foresee";
const std::string saveImg_file = param_dir + "saveImg";
const std::string speed_file = param_dir + "speed";
const std::string servo_mid_file = param_dir + "servoMid";
const std::string diff_enable_file = param_dir + "diffEnable";
const std::string diff_gain_file = param_dir + "diffGain";
const std::string diff_limit_file = param_dir + "diffLimit";
const std::string diff_threshold_file = param_dir + "diffThreshold";
const std::string diff_sign_file = param_dir + "diffSign";

double readDoubleFromFile(const std::string& filename);
bool readFlag(const std::string& filename);

extern std::atomic<double> PID_rotate;

extern double target_speed;
extern int servo_mid;

#endif
