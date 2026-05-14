#include <array>
#include <algorithm>

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

constexpr unsigned int TIME_STEP = 32;
constexpr double DT = TIME_STEP / 1000.0; // in seconds
constexpr double MIN_SPEED = -6.28;
constexpr double MAX_SPEED = 6.28;
constexpr double WALL_THRESHOLD = 100;
constexpr double BASE_SPEED = 3.14;

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using velocity = double;

std::pair<velocity, velocity>
adjust_speeds(double output)
{

    velocity l_speed = std::clamp(BASE_SPEED - output, MIN_SPEED, MAX_SPEED);

    velocity r_speed = std::clamp(BASE_SPEED + output, MIN_SPEED, MAX_SPEED);

    return {l_speed, r_speed};
}

class PID_Controller
{
public:
    explicit PID_Controller(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_err_(0.0), sum_err_(0.0) {}

    double step(double setpoint, double measured, double dt)
    {
        double err = setpoint - measured;

        sum_err_ += err * dt;

        double derivative_err = (err - prev_err_) / dt;
        prev_err_ = err;

        return kp_ * err +
               ki_ * sum_err_ +
               kd_ * derivative_err;
    }

private:
    double prev_err_;
    double sum_err_;

    double const kp_;
    double const ki_;
    double const kd_;
};

// entry point of the controller
int main(int argc, char **argv)
{
    // create the Robot instance
    Robot robot;

    // ==================================== initialize devices ====================================
    // init motors
    Motor *lm = robot.getMotor("left wheel motor");
    Motor *rm = robot.getMotor("right wheel motor");

    lm->setPosition(INFINITY);
    rm->setPosition(INFINITY);

    // init cam
    Camera *cam = robot.getCamera("camera");
    cam->enable(TIME_STEP);

    // init side sensors
    std::array<DistanceSensor *, 8> ps{
        robot.getDistanceSensor("ps0"), // forward right
        robot.getDistanceSensor("ps1"), // top right
        robot.getDistanceSensor("ps2"), // right
        robot.getDistanceSensor("ps3"), // bottom right
        robot.getDistanceSensor("ps4"), // bottom left
        robot.getDistanceSensor("ps5"), // left
        robot.getDistanceSensor("ps6"), // top left
        robot.getDistanceSensor("ps7"), // forward left
    };

    for (auto *sensor : ps)
    {
        sensor->enable(TIME_STEP);
    }

    constexpr double kp = 0.5;
    constexpr double ki = 0.5;
    constexpr double kd = 0.5;
    PID_Controller ctrl{kp, ki, kd};

    // ==================================== main loop ====================================

    while (robot.step(TIME_STEP) != -1)
    {
        // Measure distance from wall
        double dist_left = ps[5]->getValue();

        // Compute output given setpoint, error, and time increment dt
        double output = ctrl.step(WALL_THRESHOLD, dist_left, TIME_STEP);

        auto [l_speed, r_speed] = adjust_speeds(output);

        lm->setVelocity(l_speed);
        rm->setVelocity(r_speed);
    }

    return 0;
}