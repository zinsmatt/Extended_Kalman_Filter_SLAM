#include "ekf.h"
#include "config.h"
/**
 * Extended Kalman filter
 *
 * Prediction step:
 *  - predict the new state using the motion model
 *  - linearize the motion model with the first degree Taylor expansion. For that, we need to calculate its jacobian
 *  - predict the new covariance matrix (this includes the motion error)
 *
 * Correction step:
 * - For each feature observed:
 *      - if it's the first time we observe this feature, its state is initialized with this observation
 *      - get the feature from the previous state
 *      ...
 *
 *      - compute the gain matrix
 *      - adjust the state to "go" in the direction of the obervation with a weighted sum of (feature_observed - feature_pred)
 *      - adjust the covariance matrix using the gain *
 *
 */



namespace {

float normalized_angle(float a)
{
  if (a > PI)
    a -= 2 * PI;
  else if (a < -PI)
    a += 2 * PI;
  return a;
}

};

EKF::EKF(int nb_landmarks) :
  n_landmarks(nb_landmarks),
  already_observed(std::vector<bool>(nb_landmarks, false))
{
}

void EKF::init(const Robot &robot)
{
  const int N = 3 + n_landmarks * 2;
  mu = Eigen::MatrixXf::Zero(N, 1);
  cov = Eigen::MatrixXf::Zero(N, N);
  std::cout << "EKF initialized " << "\n";
  std::cout << "mu = " << mu.size() << "\n";
  std::cout << "cov = " << cov.size() << "\n";

  // initialize state vector
  mu(0) = robot.x();
  mu(1) = robot.y();
  mu(2) = robot.orientation();

  // initialize covariance matrix (robot position is certain at start)
  for (int i = 3; i < N; ++i)
  {
    cov(i, i) = COV_INF;
  }
}


void EKF::update(float v, float w, float dt, std::vector<Feature> const& features)
{
  const float EPS = 1e-4f;
  const int N = 3 + 2 * n_landmarks;

  // maps the robot motion to N * N
  Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3, N);
  F(0, 0) = 1;
  F(1, 1) = 1;
  F(2, 2) = 1;

  Eigen::MatrixXf mu_pred = mu;
  Eigen::MatrixXf cov_pred;   //
  Eigen::MatrixXf G;      // motion Jacobian
  Eigen::Matrix<float, 3, 2> V;     // motion noise Jacobian

  // Motion noise
  // This assumes a zero-mean Gaussian noise on the commands v and w
  Eigen::Matrix2f M = Eigen::Matrix2f::Zero();
  Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
   M(0, 0) = std::pow(ALPHA1 * std::abs(v) + ALPHA2 * std::abs(w), 2.0f);
   M(1, 1) = std::pow(ALPHA3 * std::abs(v) + ALPHA4 * std::abs(w), 2.0f);

  // Measurement noise
  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  Q(0, 0) = SENSOR_RANGE_NOISE_STDDEV * SENSOR_RANGE_NOISE_STDDEV;
  Q(1, 1) = SENSOR_BEARING_NOISE_STDDEV* SENSOR_BEARING_NOISE_STDDEV;

  if (std::abs(w) > EPS)
  {
    // in case of angular velocity
    float r = v / w;
    float motion_x = -r * std::sin(mu(2)) + r * std::sin(mu(2) + w * dt);
    float motion_y = r * std::cos(mu(2)) - r * std::cos(mu(2) + w * dt);
    float delta_orientation = w * dt;

    mu_pred(0) += motion_x;
    mu_pred(1) += motion_y;
    mu_pred(2) += delta_orientation;

    // motion noise jacobian
    V(0, 0) = (-std::sin(mu(2)) + std::sin(mu(2) + w * dt)) / w;
    V(0, 1) = v * (std::sin(mu(2)) - std::sin(mu(2) + w * dt)) / (w * w) + v * std::cos(mu(2) + w * dt) * dt / w;
    V(1, 0) = (std::cos(mu(2)) - std::cos(mu(2) + w * dt)) / w;
    V(1, 1) = -v * (std::cos(mu(2)) - std::cos(mu(2) + w * dt)) / (w * w) + v * std::sin(mu(2) + w * dt) * dt / w;
    V(2, 0) = 0;
    V(2, 1) = dt;
    R = V * M * V.transpose();      // create the motion noise covariance

    Eigen::Matrix3f g = Eigen::Matrix3f::Zero();
    g(0, 2) = -r * std::cos(mu(2)) + r * std::cos(mu(2) + w * dt);
    g(1, 2) = -r * std::sin(mu(2)) + r * std::sin(mu(2) + w * dt);

    G = Eigen::MatrixXf::Identity(N, N) + F.transpose() * g * F;

    cov_pred = G * cov * G.transpose() + F.transpose() * R * F;
  }
  else
  {
    // no angular velocity
    float motion_x = v * dt * std::cos(mu(2));
    float motion_y = v * dt * std::sin(mu(2));

    mu_pred(0) += motion_x;
    mu_pred(1) += motion_y;

    // motion noise jacobian
    V(0, 0) = std::cos(mu(2)) * dt;
    V(1, 0) = std::sin(mu(2)) * dt;
    V(2, 0) = 0;
    V(0, 1) = 0;
    V(1, 1) = 0;
    V(2, 1) = dt;   // not sure
    R = V * M * V.transpose();        // create the motion noise covariance

    Eigen::Matrix3f g = Eigen::Matrix3f::Zero();
    g(0, 2) = -v * dt * std::sin(mu(2));
    g(1, 2) = v * dt * std::cos(mu(2));

    G = Eigen::MatrixXf::Identity(N, N) + F.transpose() * g * F;
    cov_pred = G * cov * G.transpose() + F.transpose() * R * F;
  }


    // ****************************
    // Correction step
    // ****************************
  Eigen::MatrixXf mu_correction = Eigen::MatrixXf::Zero(mu.rows(), mu.cols());
  Eigen::MatrixXf cov_correction = Eigen::MatrixXf::Zero(cov.rows(), cov.cols());
    // for each feature
    for (auto const& f : features)
    {
      int j = f.id;
//      std::cout << "feature " << j << std::endl;
//      std::cout << "range bearing  = " << f.range << " " << f.bearing << std::endl;

      int landmark_idx = 3 + 2 * j;
      if (already_observed[j] == false)
      {
//        std::cout << "feature first time observed" << std::endl;
        // better estimate than (0, 0, 0) when the landmark is observed for the first time
        mu_pred(landmark_idx) = mu_pred(0) + f.range * std::cos(f.bearing + mu_pred(2));
        mu_pred(landmark_idx + 1) = mu_pred(1) + f.range * std::sin(f.bearing + mu_pred(2));
        already_observed[j] = true;
//        std::cout << "init features state " << mu_pred(landmark_idx) << " " << mu_pred(landmark_idx+1) << std::endl;
      }
//      std::cout << "mu_pred = " << mu_pred(0) << " " << mu_pred(1) << " " << mu_pred(2) << std::endl;
//      std::cout << "aldnmark pred = " << mu_pred(landmark_idx) << " " << mu_pred(landmark_idx+1) << std::endl;

      // vector between the estimated robot position and the estimated landmark position
      Eigen::Vector2f d(mu_pred(landmark_idx) - mu_pred(0),
                        mu_pred(landmark_idx + 1) - mu_pred(1));

      float q = d.transpose().dot(d);

//      std::cout << "delta_robot_ladnmark " << delta_robot_landmark.x() << " " << delta_robot_landmark.y() << "\n";
      // predicted observation based on the predicted robot new state (mu_pred)
      Eigen::Vector2f z_pred(std::sqrt(q),
                             normalized_angle(std::atan2(d.y(), d.x()) - mu_pred(2)));
//      std::cout << "z pred " << z_pred << std::endl;
      // Fxj maps the lower dim h to H
      Eigen::MatrixXf Fxj = Eigen::MatrixXf::Zero(5, 3 + 2 * n_landmarks);
      Fxj(0, 0) = 1;
      Fxj(1, 1) = 1;
      Fxj(2, 2) = 1;
      Fxj(3, landmark_idx) = 1;
      Fxj(4, landmark_idx + 1) = 1;

      Eigen::Matrix<float, 2, 5> h;
      float inv_sqrt_scale = 1.0f / std::sqrt(q);
      float inv_scale = 1.0f / q;
      h << -d.x() * inv_sqrt_scale, -d.y() * inv_sqrt_scale, 0,
            d.x() * inv_sqrt_scale, d.y() * inv_sqrt_scale,
            d.y() * inv_scale, -d.x() * inv_scale, -1,
           -d.y() * inv_scale, d.x() * inv_scale;
      Eigen::MatrixXf H = h * Fxj;
//      std::cout << "H=\n" << H << std::endl;
      std::cout << "cov_pred \n" << cov_pred << std::endl;
      std::cout << "temp \n" <<  (H * cov_pred * H.transpose() + Q) << "\n";
      Eigen::MatrixXf K = cov_pred * H.transpose() * (H * cov_pred * H.transpose() + Q).inverse();

//      std::cout << "K=\n" << K << std::endl;

      mu_pred += K * (Eigen::Vector2f(f.range, f.bearing) - z_pred);
      //std::cout << "mu_pred\n" << mu_pred << std::endl;
//      cov_correction += K * H;
      cov_pred = (Eigen::MatrixXf::Identity(cov.rows(), cov.cols()) - K * H) * cov_pred;
//      std::cout << H << "\n";
    }
//    std::cout << "mu_correction \n" << mu_correction << "\n";
//    std::cout << "cov_correction \n" << cov_correction << "\n";

//    mu = mu_pred + mu_correction;
//    cov = (Eigen::MatrixXf::Identity(cov.rows(), cov.cols()) - cov_correction) * cov_pred;
    mu = mu_pred;
    cov = cov_pred;
//    std::cout << "cov_pred \n" << cov_pred << "\n";
//    std::cout << "cov\n" << cov << "\n";
//    std::cout << "mu\n" << mu << "\n";
    std::cout << "=========================================================\n" << std::endl;
    //std::cout << "mu = " << mu(0) << " " << mu(1) << " " << mu(2) << std::endl;

}
