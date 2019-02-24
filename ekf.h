#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include "feature.h"
#include "robot.h"

class EKF
{
public:
  EKF(int nb_landmarks);

private:
  void init(Robot const& robot);

  void update(float v, float w, float dt, std::vector<Feature> const& features);

private:
  Eigen::MatrixXf mu;
  Eigen::MatrixXf cov;
  int n_landmarks;

  std::vector<bool> already_observed;

};

#endif // EKF_H
