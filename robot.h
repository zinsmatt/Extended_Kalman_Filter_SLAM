#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>

#define TO_DEGREES(x) x * 57.2957795131f
#define TO_RADIANS(x) x * 0.01745329251f

const float ROBOT_FOV = TO_RADIANS(60);
const float ROBOT_RANGE_VIEW = 150;

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
  float& orientation() { return orientation_; }

  float fov() const { return fov_; }
  float range() const { return range_; }


  void move(float dist);

 private:
  Eigen::Vector2f pos_;
  float orientation_;
  float fov_ = ROBOT_FOV;
  float range_ = ROBOT_RANGE_VIEW;
};

#endif // ROBOT_H
