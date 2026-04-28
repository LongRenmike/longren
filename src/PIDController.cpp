#include "PIDController.h"

#include <algorithm>

// 构造函数，初始化 PID 参数
PIDController::PIDController(double kp, double ki, double kd, double target, PIDMode mode,
    double output_limit, double integral_limit)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , target_(target)
    , prev_error_(0.0)
    , prev_prev_error_(0.0)
    , integral_(0.0)
    , prev_output_(0.0)
    , mode_(mode)
    , output_limit_(output_limit)
    , integral_limit_(integral_limit)
{
}

// 更新 PID 控制器
double PIDController::update(double measured_value)
{
    // 计算误差
    double error = target_ - measured_value;

    if (mode_ == POSITION) {
        // 位置式 PID 计算
        return positionPID(error);
    } else {
        // 增量式 PID 计算
        return incrementalPID(error);
    }
}

// 设置新的目标值
void PIDController::setTarget(double target)
{
    target_ = target;
}

// 设置 PID 参数
void PIDController::setPID(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// 设置 PID 控制模式（位置式或增量式）
void PIDController::setMode(PIDMode mode)
{
    mode_ = mode;
}

// 设置输出和积分的限幅值
void PIDController::setLimits(double output_limit, double integral_limit)
{
    output_limit_ = output_limit;
    integral_limit_ = integral_limit;
}

// 位置式 PID 实现
double PIDController::positionPID(double error)
{
    // 计算积分项，进行积分饱和处理
    integral_ += error;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    // 计算微分项
    double derivative = (error - prev_error_);

    // 计算输出
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 输出饱和处理
    output = std::clamp(output, -output_limit_, output_limit_);

    // 存储当前误差，用于下次计算
    prev_error_ = error;

    return output;
}

// 增量式 PID 实现
double PIDController::incrementalPID(double error)
{
    // 增量输出计算公式
    double delta_output = kp_ * (error - prev_error_) + ki_ * error + kd_ * (error - 2 * prev_error_ + prev_prev_error_);

    // 更新误差历史
    prev_prev_error_ = prev_error_;
    prev_error_ = error;

    // 累加增量得到新的输出
    prev_output_ += delta_output;

    // 输出饱和处理
    prev_output_ = std::clamp(prev_output_, -output_limit_, output_limit_);

    return prev_output_;
}
