#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "data_logger.hpp"

constexpr double MIN_SPEED = -6.28;
constexpr double MAX_SPEED = 6.28;
constexpr double MAX_SPEED_LEFT = 6.0;
constexpr double WALL_THRESHOLD = 80;
constexpr double BASE_SPEED = 3.14;
constexpr double KI_BUDGET = 0.3; // Want integral term to contribute up to 30% of of output

class PID_Controller
{
public:
    explicit PID_Controller(double kp, double ki, double kd, double dt, double alpha = 1.0) : kp_(kp), ki_(ki), kd_(kd), dt_(dt), alpha_(alpha), prev_err_(0.0), sum_err_(0.0), filtered_deriv_(0.0) {}

    PIDState step(double setpoint, double measured)
    {
        double err = setpoint - measured;

        if (ki_ != 0.0)
        {
            double windup_limit = (MAX_SPEED_LEFT - BASE_SPEED) * KI_BUDGET / ki_;
            sum_err_ = std::clamp(sum_err_ + err * dt_, -windup_limit, windup_limit);
        }
        else
        {
            sum_err_ = 0.0;
        }

        double raw_deriv = (err - prev_err_) / dt_;
        filtered_deriv_ = alpha_ * raw_deriv + (1.0 - alpha_) * filtered_deriv_;
        prev_err_ = err;

        

        double output = kp_ * err +
                        ki_ * sum_err_ +
                        kd_ * filtered_deriv_;

        return PIDState{
            0,
            setpoint,
            measured,
            err,
            sum_err_,
            filtered_deriv_,
            output,
            0,
            0,
            0};
    }

private:
    double const kp_;
    double const ki_;
    double const kd_;
    double const dt_;
    double const alpha_;

    double prev_err_;
    double sum_err_;
    double filtered_deriv_;
};

#endif // PID_CONTROLLER_HPP