#include <array>
#include <algorithm>
#include <iostream>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include "pid.hpp"

constexpr unsigned int TIME_STEP = 24;
constexpr double DT = TIME_STEP / 1000.0;

// ================================ turn constants ================================
constexpr double TURN_SPEED = 2.0;
constexpr int TURN_STEPS = 65; // calibrate empirically for 90 degrees

using namespace webots;
using velocity = double;

// ================================ state machine ================================
enum class RobotState
{
    FOLLOW_WALL,
    TURN_LEFT,
    TURN_RIGHT,
    GO_FORWARD,
};

// ================================ sensor reading ================================
struct SensorReading
{
    double front_left;
    double front_right;
    double side_left;
    double side_right;

    bool wall_ahead() const
    {
        return front_left > FRONT_THRESHOLD || front_right > FRONT_THRESHOLD;
    }
    bool wall_left() const { return side_left > WALL_THRESHOLD; }
    bool wall_right() const { return side_right > WALL_THRESHOLD; }
};

SensorReading read_sensors(std::array<DistanceSensor *, 8> const &ps)
{
    return {
        ps[7]->getValue(), // front left
        ps[0]->getValue(), // front right
        ps[5]->getValue(), // side left  — PID sensor
        ps[2]->getValue(), // side right
    };
}

// ================================ speed helpers ================================
std::pair<velocity, velocity> wall_follow_speeds(double output)
{
    velocity l = std::clamp(BASE_SPEED - output, MIN_SPEED, MAX_SPEED_LEFT);
    velocity r = std::clamp(BASE_SPEED + output, MIN_SPEED, MAX_SPEED);
    return {l, r};
}

std::pair<velocity, velocity> turn_speeds(RobotState state)
{
    if (state == RobotState::TURN_LEFT)
        return {-TURN_SPEED, TURN_SPEED};
    return {TURN_SPEED, -TURN_SPEED};
}

// ================================ state transitions ================================
RobotState next_state(RobotState current, SensorReading const &s)
{
    switch (current)
    {
    case RobotState::FOLLOW_WALL:
        if (s.wall_ahead() && s.wall_left())
            return RobotState::TURN_RIGHT;
        if (s.wall_ahead())
            return RobotState::TURN_LEFT;
        if (!s.wall_left())
            return RobotState::GO_FORWARD;
        return RobotState::FOLLOW_WALL;

    case RobotState::GO_FORWARD:
        if (s.wall_left())
            return RobotState::FOLLOW_WALL;
        if (s.wall_ahead())
            return RobotState::TURN_LEFT;
        return RobotState::GO_FORWARD;

    // turn completion is handled separately in the main loop
    case RobotState::TURN_LEFT:
    case RobotState::TURN_RIGHT:
        return current;
    }
    return current;
}

// ================================ entry point ================================
int main(int argc, char **argv)
{
    Supervisor *robot = new Supervisor();

    // init camera
    Camera *cam = robot->getCamera("camera");
    cam->enable(TIME_STEP);

    // init sensors
    std::array<DistanceSensor *, 8> ps{
        robot->getDistanceSensor("ps0"), // front right
        robot->getDistanceSensor("ps1"), // top right
        robot->getDistanceSensor("ps2"), // right
        robot->getDistanceSensor("ps3"), // bottom right
        robot->getDistanceSensor("ps4"), // bottom left
        robot->getDistanceSensor("ps5"), // left
        robot->getDistanceSensor("ps6"), // top left
        robot->getDistanceSensor("ps7"), // front left
    };
    for (auto *s : ps)
        s->enable(TIME_STEP);

    // init motors
    Motor *lm = robot->getMotor("left wheel motor");
    Motor *rm = robot->getMotor("right wheel motor");
    lm->setPosition(INFINITY);
    rm->setPosition(INFINITY);

    // init PID
    PID_Controller ctrl{KP, KI, KD, DT, ALPHA};

    // state machine
    RobotState state = RobotState::FOLLOW_WALL;
    int turn_steps = 0;

    while (robot->step(TIME_STEP) != -1)
    {
        auto sensors = read_sensors(ps);

        std::cout << sensors.side_left << "\n";

        switch (state)
        {
        case RobotState::FOLLOW_WALL:
        {
            double output = ctrl.step(WALL_THRESHOLD, sensors.side_left);
            auto [l, r] = wall_follow_speeds(output);
            lm->setVelocity(l);
            rm->setVelocity(r);
            state = next_state(state, sensors);
            // reset PID when leaving FOLLOW_WALL
            if (state != RobotState::FOLLOW_WALL)
                ctrl.reset();
            break;
        }
        case RobotState::TURN_LEFT:
        case RobotState::TURN_RIGHT:
        {
            auto [l, r] = turn_speeds(state);
            lm->setVelocity(l);
            rm->setVelocity(r);
            if (++turn_steps >= TURN_STEPS)
            {
                turn_steps = 0;
                state = RobotState::GO_FORWARD; // re-acquire wall before PID
            }
            break;
        }
        case RobotState::GO_FORWARD:
        {
            lm->setVelocity(BASE_SPEED);
            rm->setVelocity(BASE_SPEED);
            state = next_state(state, sensors);
            break;
        }
        }
    }

    return 0;
}