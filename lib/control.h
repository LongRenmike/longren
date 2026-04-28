
#ifndef CONTROL_H
#define CONTROL_H

#include <unistd.h>
#include <atomic>

void ControlInit();
void ControlMain();
void ControlExit();

extern double angle;
extern std::atomic<bool> zebra_signal;
extern std::atomic<bool> traffic_stop_signal;

extern double mortor_kp;
extern double mortor_ki;
extern double mortor_kd;

#endif
