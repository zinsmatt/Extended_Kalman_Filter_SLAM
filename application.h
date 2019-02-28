#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

#include "config.h"
#include "ekf.h"
#include "feature.h"

class Landmark;
class Robot;

class Application
{
public:
  Application(unsigned int w, unsigned int h, std::string const& str);
  ~Application();

  void exec();

  void init_robot(float x, float y, float angle);
  void add_landmark(float x, float y);

private:
  void render_robot();
  void render_landmarks(std::vector<Feature> const& observed_features);
  void render_estimated_robot(Eigen::Vector3f const& pose);
  void render_estimated_landmarks(Eigen::MatrixXf const& landmarks);
  void render_robot_ellipse(EKF const& ekf);
  void render_landmarks_ellipse(EKF const& ekf);
  std::tuple<float, float, float> ellipse_from_cov(Eigen::Matrix2f const& cov);
  void draw_ellipse(Eigen::Vector2f const& position, std::tuple<float, float, float> ellipse, const sf::Color &color);


private:
  sf::RenderWindow win;
  unsigned int width, height;
  std::string name;

  std::unique_ptr<Robot> robot;
  std::vector< std::unique_ptr<Landmark> > landmarks;

  const unsigned int FPS = 30;      // this should not be changed. Otherwise all the control parameters will need to be adjusted
  const float DT = 1.0f / FPS;
  // Graphics parameters
  const int LANDMARK_SIZE = 10;
  const sf::Color LANDMARK_COLOR = sf::Color::White;
  const sf::Color LANDMARK_OBSERVED_COLOR = sf::Color::Red;
  const sf::Color LANDMAKR_ELLIPSE_COLOR = sf::Color(80, 80, 80);

  const int ROBOT_SIZE = 10;
  const int ROBOT_THICKNESS = 2;
  const int ROBOT_VIEW_THICKNESS = 2;
  const int ELLIPSE_THICKNESS = 1;
  const unsigned int ELLIPSE_NB_POINTS = 64;
  const sf::Color ROBOT_COLOR = sf::Color::Green;
  const sf::Color ROBOT_VIEW_COLOR = sf::Color::Yellow;
  const sf::Color ROBOT_ESTIMATED_COLOR = sf::Color::Blue;
  const sf::Color ROBOT_ELLIPSE_COLOR = sf::Color(80, 80, 80);


  const sf::Color BACKGROUND_COLOR = sf::Color::Black;


};

#endif // WINDOW_H
