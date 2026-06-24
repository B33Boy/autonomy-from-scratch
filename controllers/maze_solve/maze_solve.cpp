#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include "pid.hpp"

constexpr unsigned int TIME_STEP = 24;
constexpr double DT = TIME_STEP / 1000.0;
constexpr double M_PI = 3.14159;

// ================================ turn constants ================================
constexpr double TURN_SPEED = 2.0;

// e-puck geometry — used to compute how far each wheel must rotate for a 90° pivot.
// For any other platform, swap in the correct values from the robot's spec sheet.
//
//   arc each wheel travels = (AXLE_LENGTH / 2) × (π / 2)
//   wheel rotation (rad)   = arc / WHEEL_RADIUS
//
constexpr double WHEEL_RADIUS = 0.0205; // metres
constexpr double AXLE_LENGTH = 0.052;   // metres, centre-to-centre
constexpr double TURN_RADIANS =
    (AXLE_LENGTH / 2.0) * (M_PI / 2.0) / WHEEL_RADIUS; // ≈ 1.99 rad per 90°

constexpr double WALL_SETPOINT = 80.0; // desired side sensor reading

using namespace webots;
using velocity = double;

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
    bool wall_left() const { return side_left > SIDE_THRESHOLD; }
    bool wall_right() const { return side_right > SIDE_THRESHOLD; }
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

// ================================ encoder turn primitives =====================
//
// Instead of counting simulation steps (open-loop, time-dependent),
// we read absolute wheel positions from the position sensors and spin
// until each wheel has physically rotated TURN_RADIANS from where it
// started.  This is robust to speed variation and simulation jitter.
//
// NOTE: Webots position sensors return a monotonically increasing angle
// (unbounded, not wrapped to 2π), so simple subtraction always gives
// the correct delta — no wrap-around handling needed.

void turn_right(Supervisor *robot,
                Motor *lm, Motor *rm,
                PositionSensor *lps, PositionSensor *rps)
{
    double const start_l = lps->getValue();
    double const start_r = rps->getValue();

    lm->setVelocity(TURN_SPEED);  // left wheel forward
    rm->setVelocity(-TURN_SPEED); // right wheel backward

    while (robot->step(TIME_STEP) != -1)
    {
        double delta_l = std::abs(lps->getValue() - start_l);
        double delta_r = std::abs(rps->getValue() - start_r);
        if (delta_l >= TURN_RADIANS && delta_r >= TURN_RADIANS)
            break;
    }

    lm->setVelocity(0.0);
    rm->setVelocity(0.0);
}

void turn_left(Supervisor *robot,
               Motor *lm, Motor *rm,
               PositionSensor *lps, PositionSensor *rps)
{
    double const start_l = lps->getValue();
    double const start_r = rps->getValue();

    lm->setVelocity(-TURN_SPEED); // left wheel backward
    rm->setVelocity(TURN_SPEED);  // right wheel forward

    while (robot->step(TIME_STEP) != -1)
    {
        double delta_l = std::abs(lps->getValue() - start_l);
        double delta_r = std::abs(rps->getValue() - start_r);
        if (delta_l >= TURN_RADIANS && delta_r >= TURN_RADIANS)
            break;
    }

    lm->setVelocity(0.0);
    rm->setVelocity(0.0);
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

    // init position sensors (encoders)
    // Webots exposes these from the motor; enable them at the same timestep.
    PositionSensor *lps = lm->getPositionSensor();
    PositionSensor *rps = rm->getPositionSensor();
    lps->enable(TIME_STEP);
    rps->enable(TIME_STEP);

    // init PID
    PID_Controller ctrl{KP, KI, KD, DT, ALPHA};

    // Step once so sensor buffers are populated before the main loop reads them
    robot->step(TIME_STEP);

    // ================================ main loop ================================
    /**
    WHILE NOT at the exit:
        IF no wall on the left:
            Turn Left
            Move Forward
        ELSE IF no wall in front:
            Move Forward
        ELSE IF no wall on the right:
            Turn Right
            Move Forward
        ELSE:
            Turn Around (Dead End)
     */

    while (robot->step(TIME_STEP) != -1)
    {
        auto sensors = read_sensors(ps);
        std::cout << "wall_left: " << sensors.wall_left() << " " << "wall_right: " << sensors.wall_ahead() << "\n";

        // --- motion planning hook (fill this in next) ---
        if (!sensors.wall_left())
        {
            ctrl.reset();
            turn_left(robot, lm, rm, lps, rps);
        }
        else if (!sensors.wall_ahead())
        {
            double pid_out = ctrl.step(WALL_SETPOINT, sensors.side_left);
            auto [lv, rv] = wall_follow_speeds(pid_out);
            lm->setVelocity(lv);
            rm->setVelocity(rv);
        }
        else if (!sensors.wall_right())
        {
            ctrl.reset();
            turn_right(robot, lm, rm, lps, rps);
        }
        else
        {
            // turn around
            ctrl.reset();
            turn_left(robot, lm, rm, lps, rps);
            turn_left(robot, lm, rm, lps, rps);
        }
        // else
        // {
        //     // Wall-follow tick
        //     double pid_out = ctrl.step(WALL_SETPOINT, sensors.side_left);
        //     auto [lv, rv] = wall_follow_speeds(pid_out);
        //     lm->setVelocity(lv);
        //     rm->setVelocity(rv);
        // }
    }

    return 0;
}