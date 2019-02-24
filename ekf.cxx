#include "ekf.h"


EKF::EKF(int nb_landmarks) :
  n_landmarks(nb_landmarks),
  already_observed(std::vector<bool>(nb_landmarks, false))
{
}

void EKF::init(const Robot &robot)
{
  int n = 3 + n_landmarks * 3;
  mu = Eigen::MatrixXf::Zero(n, 1);
  cov = Eigen::MatrixXf::Ones(n, n) * 1e10;

  // initialize state vector
  mu(0) = robot.x();
  mu(1) = robot.y();
  mu(2) = robot.orientation();

  // initialize covariance matrix (robot position is certain at start)
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      cov(i, j) = 0;
    }
  }
}

const float SENSOR_RANGE_NOISE = 0.01f;
const float SENSOR_BEARING_NOISE = 0.001f;

void EKF::update(float v, float w, float dt, std::vector<Feature> const& features)
{
  const float EPS = 1e-4f;
  Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3, 3 * n_landmarks);
  F(0, 0) = 1;
  F(1, 1) = 1;
  F(2, 2) = 1;

  Eigen::MatrixXf mu_pred = mu;
  Eigen::MatrixXf cov_pred;   //
  Eigen::MatrixXf G;      // motion Jacobian

  Eigen::Matrix3f Q = Eigen::Matrix3f::Zero();
  Q(0, 0) = SENSOR_RANGE_NOISE;
  Q(1, 1) = SENSOR_BEARING_NOISE;
  Q(2, 2) = 0; //SENSOR_SIGNATURE_NOISE

  if (std::abs(w) > EPS)
  {
    float r = std::abs(v / w);
    float motion_x = -r * std::sin(mu(2)) + r * std::sin(mu(2) + w * dt);
    float motion_y = r * std::cos(mu(2)) - r * std::cos(mu(2) + w * dt);
    float delta_orientation = w * dt;

    mu_pred(0) += motion_x;
    mu_pred(1) += motion_y;
    mu_pred(2) += delta_orientation;

    Eigen::Matrix3f g = Eigen::Matrix3f::Zero();
    g(0, 2) = -r * std::cos(mu(2)) + r * std::cos(mu(2) + w * dt);
    g(1, 2) = -r * std::sin(mu(2)) + r * std::sin(mu(2) + w * dt);

    G = Eigen::MatrixXf::Identity(n_landmarks, n_landmarks) + F.transpose() * g * F;

    cov_pred = G.transpose() * cov * G;    // no motion noise for now

    // for each feature
    for (auto const& f : features)
    {
      int j = f.id;

      int landmark_idx = 3 + 3 * j;
      if (already_observed[j] == false)
      {
        // better estimate than (0, 0, 0) when the landmark is observed for the first time
        mu_pred(landmark_idx) = mu_pred(0) + f.range * std::cos(f.bearing + mu_pred(2));
        mu_pred(landmark_idx + 1) = mu_pred(1) + f.range * std::sin(f.bearing + mu_pred(2));
        mu_pred(landmark_idx + 2) = f.s;
        already_observed[j] = true;
      }

      Eigen::Vector2f delta_robot_landmark(mu_pred(landmark_idx) - mu_pred(0),
                                           mu_pred(landmark_idx + 1) - mu_pred(1));
      float q = delta_robot_landmark.transpose().dot(delta_robot_landmark);

      // predicted observation based on the predicted robot new state (mu_pred)
      Eigen::Vector3f z_pred(std::sqrt(q),
                             std::atan2(delta_robot_landmark.y(), delta_robot_landmark.x()) - mu_pred(2),
                             mu_pred(landmark_idx + 2));
      // Fxj maps the lower dim h to H
      Eigen::MatrixXf Fxj = Eigen::MatrixXf::Zero(6, 3 + 3 * n_landmarks);
      Fxj(0, 0) = 1;
      Fxj(1, 1) = 1;
      Fxj(2, 2) = 1;
      Fxj(3, landmark_idx) = 1;
      Fxj(4, landmark_idx + 1) = 1;
      Fxj(5, landmark_idx + 2) = 1;

      Eigen::Matrix<float, 3, 6> h;
      float inv_sqrt_scale = 1.0f / std::sqrt(q);
      float inv_scale = 1.0f / q;
      h << -delta_robot_landmark.x() * inv_sqrt_scale, -delta_robot_landmark.y() * inv_sqrt_scale, 0,
           delta_robot_landmark.x() * inv_sqrt_scale, delta_robot_landmark.y() * inv_sqrt_scale, 0,
           delta_robot_landmark.y() * inv_scale, -delta_robot_landmark.x() * inv_scale, -1,
           -delta_robot_landmark.y() * inv_scale, delta_robot_landmark.x() * inv_scale, 0,
           0, 0, 0, 0, 0, 1;
      Eigen::MatrixXf H = h * Fxj;
      Eigen::MatrixXf K = cov_pred * H.transpose() * (H * cov_pred * H.transpose() + Q).inverse();

      mu_pred += K * (Eigen::Vector3f(f.range, f.bearing, f.s) - z_pred);
      cov_pred = (Eigen::MatrixXf::Identity(3 + 3 * n_landmarks, 3 + 3 * n_landmarks) - K * H) * cov_pred;
    }
    mu = mu_pred;
    cov = cov_pred;
  }

}
