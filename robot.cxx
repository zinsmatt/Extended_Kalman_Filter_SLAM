#include "robot.h"


Robot::Robot(float x, float y, float angle) :
  pos_(x, y), orientation_(angle)
{
}

void Robot::move(float dist)
{
  pos_.x() += std::cos(orientation_) * dist;
  pos_.y() += std::sin(orientation_) * dist;
}
