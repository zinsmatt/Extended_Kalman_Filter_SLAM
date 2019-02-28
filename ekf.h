#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include "feature.h"
#include "robot.h"

class EKF
{
public:
  EKF(int nb_landmarks);

  void init(Robot const& robot);

  void update(float v, float w, float dt, std::vector<Feature> const& features);

  Eigen::Vector3f estimated_pose() const { return mu.block<3, 1>(0, 0); }
  Eigen::MatrixXf estimated_landmarks() const { return  mu.block(3, 0, mu.rows() - 3, mu.cols()); }
  Eigen::Vector2f estimated_landmark(int i) const { return mu.block<2, 1>(3 + 2 * i, 0); }

  Eigen::Matrix2f get_robot_cov() const { return cov.block<2, 2>(0, 0); }
  Eigen::Matrix2f get_landmark_cov(int i) const { return cov.block<2, 2>(3 + i * 2, 3 + i * 2); }

private:
  Eigen::MatrixXf mu;
  Eigen::MatrixXf cov;
  int n_landmarks;

  std::vector<bool> already_observed;

};

#endif // EKF_H
