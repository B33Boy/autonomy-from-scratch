#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "data_logger.hpp"

class PID_Controller
{
public:
    explicit PID_Controller(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_err_(0.0), sum_err_(0.0) {}

    PIDState step(double dt, double setpoint, double measured)
    {
        double err = setpoint - measured;

        sum_err_ = std::clamp(sum_err_ + err * dt, -10.0, 10.0); // anti-integral windup

        double derivative_err = (err - prev_err_) / dt;
        prev_err_ = err;

        double output = kp_ * err +
                        ki_ * sum_err_ +
                        kd_ * derivative_err;

        return PIDState{
            dt,
            setpoint,
            measured,
            err,
            sum_err_,
            derivative_err,
            output,
            0,
            0};
    }

private:
    double prev_err_;
    double sum_err_;

    double const kp_;
    double const ki_;
    double const kd_;
};

#endif // PID_CONTROLLER_HPP