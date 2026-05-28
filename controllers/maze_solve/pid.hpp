#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

// ================================ robot constants ================================
constexpr double MIN_SPEED = -6.28;
constexpr double MAX_SPEED = 6.28;
constexpr double MAX_SPEED_LEFT = 6.0;
constexpr double BASE_SPEED = 3.14;

// ================================ tuned gains ================================
constexpr double KP = 0.00900;
constexpr double KI = 0.00286;
constexpr double KD = 0.00708;
constexpr double ALPHA = 0.2;

// ================================ wall following ================================
constexpr double WALL_THRESHOLD = 100.0;
constexpr double FRONT_THRESHOLD = 80.0; // separate threshold for turn decisions
constexpr double KI_BUDGET = 0.3;        // integral term gets 30% of actuator range

class PID_Controller
{
public:
    explicit PID_Controller(double kp, double ki, double kd, double dt, double alpha = 1.0)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), alpha_(alpha),
          prev_err_(0.0), sum_err_(0.0), filtered_deriv_(0.0) {}

    double step(double setpoint, double measured)
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

        return kp_ * err + ki_ * sum_err_ + kd_ * filtered_deriv_;
    }

    void reset()
    {
        prev_err_ = 0.0;
        sum_err_ = 0.0;
        filtered_deriv_ = 0.0;
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