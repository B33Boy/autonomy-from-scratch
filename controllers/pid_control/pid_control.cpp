#include <array>
#include <algorithm>
#include <iostream>
#include <format>

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include "pid.hpp"
#include "data_logger.hpp"

constexpr unsigned int TIME_STEP = 24;    // webots interprets this as ms, required by robot step loop
constexpr double DT = TIME_STEP / 1000.0; // in s, required by PID step calculation

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using velocity = double;

std::pair<velocity, velocity> adjust_speeds(double output)
{
    // Add asymmetry to the left wheel by capping max speed
    velocity l_speed = std::clamp(BASE_SPEED - output, MIN_SPEED, MAX_SPEED_LEFT);
    velocity r_speed = std::clamp(BASE_SPEED + output, MIN_SPEED, MAX_SPEED);

    return {l_speed, r_speed};
}

// entry point of the controller
int main(int argc, char **argv)
{
    // create the Supervisor instance
    Supervisor *robot = new Supervisor();

    // ==================================== initialize devices ====================================
    // init cam (for funsies, not used in this demo)
    Camera *cam = robot->getCamera("camera");
    cam->enable(TIME_STEP);

    // init side sensors
    std::array<DistanceSensor *, 8> ps{
        robot->getDistanceSensor("ps0"), // forward right
        robot->getDistanceSensor("ps1"), // top right
        robot->getDistanceSensor("ps2"), // right
        robot->getDistanceSensor("ps3"), // bottom right
        robot->getDistanceSensor("ps4"), // bottom left
        robot->getDistanceSensor("ps5"), // left
        robot->getDistanceSensor("ps6"), // top left
        robot->getDistanceSensor("ps7"), // forward left
    };

    for (auto *sensor : ps)
    {
        sensor->enable(TIME_STEP);
    }

    // init motors
    Motor *lm = robot->getMotor("left wheel motor");
    Motor *rm = robot->getMotor("right wheel motor");

    lm->setPosition(INFINITY);
    rm->setPosition(INFINITY);

    // init PID Controller
    constexpr double kp = 0.00900;
    constexpr double ki = 0.00286;
    constexpr double kd = 0.00708;
    constexpr double alpha = 0.2;
    PID_Controller ctrl{kp, ki, kd, DT, alpha};

    double running_mean = 0.0;
    int step_count = 0;
    // init logger

    std::string filename = std::format("out_data/pid_kp_{:.5f}_ki_{:.5f}_kd_{:.5f}_alpha_{:.2}.csv", kp, ki, kd, alpha);
    DataLogger lg{filename};
    std::cout << "Logging to " << filename << "\n";

    // ==================================== main loop ====================================

    while (robot->step(TIME_STEP) != -1)
    {
        double dist_left = ps[5]->getValue();
        std::cout << "dist_left: " << dist_left << "\n";

        // Perform control step and get PIDState
        auto state = ctrl.step(WALL_THRESHOLD, dist_left);
        step_count++;
        running_mean += (state.err - running_mean) / step_count;

        // Adjust speeds
        auto [l_speed, r_speed] = adjust_speeds(state.output);
        lm->setVelocity(l_speed);
        rm->setVelocity(r_speed);

        // Finally set the rest of the missing fields in state for logging
        state.time = robot->getTime();
        state.l_speed = l_speed;
        state.r_speed = r_speed;
        state.rolling_mean = running_mean;

        lg.log(state);
    }

    return 0;
}