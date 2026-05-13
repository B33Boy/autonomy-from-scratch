#include <array>

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// time in [ms] of a simulation step
constexpr int TIME_STEP = 32;

constexpr double MAX_SPEED = 6.28;

constexpr double SMALL_DELTA = 0.3;
constexpr double LARGE_DELTA = 0.5;

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using velocity = double;

std::pair<velocity, velocity>
adjust_speeds(std::array<DistanceSensor *, 3> const &gs)
{
  // read sensors outputs
  double const l_sensor = gs[0]->getValue();
  double const r_sensor = gs[2]->getValue();

  bool const l_on_line = l_sensor < 500;
  bool const r_on_line = r_sensor < 500;

  velocity l_speed{};
  velocity r_speed{};

  if (l_on_line && !r_on_line)
  {
    l_speed = -MAX_SPEED * SMALL_DELTA;
    r_speed = MAX_SPEED * LARGE_DELTA;
  }
  else if (!l_on_line && r_on_line)
  {
    l_speed = MAX_SPEED * LARGE_DELTA;
    r_speed = -MAX_SPEED * SMALL_DELTA;
  }
  else
  {
    l_speed = MAX_SPEED * LARGE_DELTA;
    r_speed = MAX_SPEED * LARGE_DELTA;
  }

  return {l_speed, r_speed};
}

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

  // init ground sensors
  std::array<DistanceSensor *, 3> gs{
      robot.getDistanceSensor("gs0"),
      robot.getDistanceSensor("gs1"),
      robot.getDistanceSensor("gs2")};

  for (DistanceSensor *sensor : gs)
    sensor->enable(TIME_STEP);

  while (robot.step(TIME_STEP) != -1)
  {
    auto [l_speed, r_speed] = adjust_speeds(gs);

    lm->setVelocity(l_speed);
    rm->setVelocity(r_speed);
  }

  return 0;
}