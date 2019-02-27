#include "robot.h"


Robot::Robot(float x, float y, float angle) :
  pos_(x, y), orientation_(angle)
{
  position_noise_ = std::normal_distribution<float>(0.0f, MOTION_NOISE_POSITON_STDDEV);
  orientation_noise_ = std::normal_distribution<float>(0.0f, MOTION_NOISE_ORIENTATION_STDDEV);
  range_noise_ = std::normal_distribution<float>(0.0f, SENSOR_RANGE_NOISE_STDDEV);
  bearing_noise_= std::normal_distribution<float>(0.0f, SENSOR_BEARING_NOISE_STDDEV);
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
  if (angular_velocity_ > 0)
    orientation_ += orientation_noise_(random_generator_);
  if (orientation_ > PI) {
    orientation_ -= 2 * PI;
  }
  if (orientation_ < -PI) {
    orientation_ += 2 * PI;
  }
  pos_.x() += std::cos(orientation_) * velocity_ * dt;
  pos_.y() += std::sin(orientation_) * velocity_ * dt;
  if (velocity_ > 0)
  {
    pos_.x() += position_noise_(random_generator_);
    pos_.y() += position_noise_(random_generator_);
  }
}

// returns the list of features observed by the robot
std::vector<Feature> Robot::get_features(const std::vector<std::unique_ptr<Landmark> > &landmarks)
{
  std::vector<Feature> features;
  for (unsigned int i = 0; i < landmarks.size(); ++i)
  {
    Eigen::Vector2f landmark_pos(landmarks[i]->x, landmarks[i]->y);
    Eigen::Vector2f v = landmark_pos - pos_;
    float d = v.norm() + range_noise_(random_generator_);
    float a = std::atan2(v.y(), v.x()) + bearing_noise_(random_generator_);
    // force the angle difference to be in [-PI, PI]
    float angle_diff = a - orientation_;
    if (angle_diff > PI)
      angle_diff -= 2 * PI;
    else if (angle_diff < -PI)
      angle_diff += 2 * PI;
    if (d <= range_ && std::abs(angle_diff) <= fov_ / 2) {
      features.push_back({d, angle_diff, landmarks[i]->s, static_cast<int>(i)});
    }
  }
  return features;
}
