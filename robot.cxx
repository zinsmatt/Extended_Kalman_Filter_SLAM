#include "robot.h"


Robot::Robot(float x, float y, float angle) :
  pos_(x, y), orientation_(angle)
{
}

void Robot::set_orientation(float angle) {
  if (angle < PI && angle > -PI) {
    orientation_ = angle;
  } else {
    std::cerr << "Invalid orientation" << std::endl;
  }
}

/// update the robot assuming a forward movement with a constant velocity and a final rotation
void Robot::update(float dt)
{
  orientation_ += angular_velocity_ * dt;
  if (orientation_ > PI) {
    orientation_ -= 2 * PI;
  }
  if (orientation_ < -PI) {
    orientation_ += 2 * PI;
  }
  pos_.x() += std::cos(orientation_) * velocity_ * dt;
  pos_.y() += std::sin(orientation_) * velocity_ * dt;
}

std::vector<bool> Robot::get_observed_landmarks(const std::vector<std::unique_ptr<Landmark> > &landmarks)
{
  std::vector<bool> is_observed(landmarks.size(), false);
  for (unsigned int i = 0; i < landmarks.size(); ++i)
  {
    Eigen::Vector2f landmark_pos(landmarks[i]->x, landmarks[i]->y);
    Eigen::Vector2f v = landmark_pos - pos_;
    float d = v.norm();
    float a = std::atan2(v.y(), v.x());
    if (d <= range_ && std::abs(orientation_ - a) <= fov_ / 2) {
      is_observed[i] = true;
    }
  }
  return is_observed;
}
