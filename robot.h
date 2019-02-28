#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>

#include "config.h"
#include "feature.h"
#include "landmark.h"

#include <iostream>
#include <memory>
#include <random>
#include <vector>

class Robot
{
public:
  /**
   * @brief Robot
   * @param x [in]
   * @param y [in]
   * @param angle [in] angle wrt x axis
   */
  Robot(float x, float y, float angle);

  float x() const { return pos_.x(); }
  float y() const { return pos_.y(); }

  Eigen::Vector2f const& pos() const { return pos_; }
  Eigen::Vector2f& pos() { return pos_; }

  float orientation() const { return orientation_; }
  void set_orientation(float angle);

  float velocity() const { return velocity_; }
  float& velocity() { return velocity_; }

  float angular_velocity() const { return angular_velocity_; }
  float& angular_velocity() { return angular_velocity_; }

  float fov() const { return fov_; }
  float range() const { return range_; }

  void update(float dt);

  std::vector<Feature> get_features(std::vector< std::unique_ptr<Landmark> >const& landmarks);

 private:
  Eigen::Vector2f pos_;
  float orientation_;
  float velocity_ = 0;
  float angular_velocity_ = 0;
  float fov_ = ROBOT_FOV;
  float range_ = ROBOT_RANGE_VIEW;

// random to simulate noise
  std::default_random_engine random_generator_;
  std::normal_distribution<float> velocity_noise_;
  std::normal_distribution<float> angular_velocity_noise_;
  std::normal_distribution<float> range_noise_;
  std::normal_distribution<float> bearing_noise_;

};

#endif // ROBOT_H
