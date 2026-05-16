#include <array>
#include <algorithm>

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include "pid_controller.hpp"

constexpr unsigned int TIME_STEP = 32;    // in ms, required by robot step loop
constexpr double DT = TIME_STEP / 1000.0; // in s, required by PID step calculation
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

// entry point of the controller
int main(int argc, char **argv)
{
    // create the Robot instance
    Robot robot;

    // ==================================== initialize devices ====================================
    // init cam (for funsies, not used in this demo)
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

    // init motors
    Motor *lm = robot.getMotor("left wheel motor");
    Motor *rm = robot.getMotor("right wheel motor");

    lm->setPosition(INFINITY);
    rm->setPosition(INFINITY);

    // init PID Controller
    constexpr double kp = 0.20;
    constexpr double ki = 0.001;
    constexpr double kd = 0.001;
    PID_Controller ctrl{kp, ki, kd};

    // init data logger
    

    // ==================================== main loop ====================================

    while (robot.step(TIME_STEP) != -1)
    {
        double dist_left = (ps[4]->getValue() + ps[5]->getValue() + ps[6]->getValue()) / 3.0;

        double output = ctrl.step(WALL_THRESHOLD, dist_left, DT);

        auto [l_speed, r_speed] = adjust_speeds(output);
        lm->setVelocity(l_speed);
        rm->setVelocity(r_speed);
    }

    return 0;
}