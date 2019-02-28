#include "application.h"

#include "landmark.h"
#include "robot.h"
#include <iostream>

Application::Application(unsigned int w, unsigned int h, std::string const& str) :
  width(w), height(h), name(str), robot(nullptr)
{
  sf::ContextSettings settings;
  settings.antialiasingLevel = 8;

  win.create(sf::VideoMode(width, height), name, sf::Style::Default, settings);
  win.setFramerateLimit(FPS);

}

Application::~Application()
{
}

void Application::exec()
{
  EKF ekf(landmarks.size());
  ekf.init(*robot);
  while (win.isOpen())
  {
    sf::Event event;
    while (win.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        win.close();
      if (event.type == sf::Event::KeyPressed)
      {
        if (event.key.code == sf::Keyboard::Escape) win.close();
        if (event.key.code == sf::Keyboard::Left) robot->angular_velocity() = -ROBOT_ROTATION_SPEED;
        if (event.key.code == sf::Keyboard::Right) robot->angular_velocity() = ROBOT_ROTATION_SPEED;
        if (event.key.code == sf::Keyboard::Up) robot->velocity() = ROBOT_MOVE_SPEED;
        if (event.key.code == sf::Keyboard::Down) robot->velocity() = -ROBOT_MOVE_SPEED;
      }
      if (event.type == sf::Event::KeyReleased)
      {
        if (event.key.code == sf::Keyboard::Left) robot->angular_velocity() = 0;
        if (event.key.code == sf::Keyboard::Right) robot->angular_velocity() = 0;
        if (event.key.code == sf::Keyboard::Up) robot->velocity() = 0;
        if (event.key.code == sf::Keyboard::Down) robot->velocity() = 0;
      }
    }
    win.clear(BACKGROUND_COLOR);

    robot->update(DT);
    std::vector<Feature> observed_features = robot->get_features(landmarks);
    ekf.update(robot->velocity(), robot->angular_velocity(), DT, observed_features);

    render_robot();
    render_landmarks(observed_features);

    Eigen::Vector3f estimated_pose = ekf.estimated_pose();
    render_estimated_robot(estimated_pose);
    Eigen::MatrixXf estimated_landmarks = ekf.estimated_landmarks();
    render_estimated_landmarks(estimated_landmarks);

    render_robot_ellipse(ekf);
    render_landmarks_ellipse(ekf);

    win.display();
  }
}

void Application::init_robot(float x, float y, float angle)
{
  robot = std::make_unique<Robot>(x, y, angle);
}

void Application::add_landmark(float x, float y)
{
  landmarks.push_back(std::make_unique<Landmark>(x, y));
}

void Application::render_robot()
{
  if (robot != nullptr)
  {
    int m = ROBOT_SIZE;
    sf::CircleShape circle(ROBOT_SIZE);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineThickness(ROBOT_THICKNESS);
    circle.setOutlineColor(ROBOT_COLOR);
    circle.setPosition(robot->x() - m, robot->y() - m);

    sf::RectangleShape rect(sf::Vector2f(ROBOT_SIZE * 2, ROBOT_THICKNESS));
    rect.setRotation(TO_DEGREES(robot->orientation()));
    rect.setFillColor(ROBOT_COLOR);
    rect.setPosition(robot->x(), robot->y());

    sf::ConvexShape convex;
    convex.setFillColor(sf::Color::Transparent);
    convex.setOutlineThickness(ROBOT_VIEW_THICKNESS);
    convex.setOutlineColor(ROBOT_VIEW_COLOR);
    size_t range_view_points = 12;
    convex.setPointCount(range_view_points + 1);
    convex.setPoint(0, sf::Vector2f(robot->x(), robot->y()));
    float angle_step = robot->fov() / (range_view_points - 1);
    float half_fov = robot->fov() / 2;
    size_t index = 1;
    for (float a = -half_fov; a <= half_fov; a += angle_step)
    {
      sf::Vector2f p(robot->x() + robot->range() * std::cos(a + robot->orientation()),
                     robot->y() + robot->range() * std::sin(a + robot->orientation()));
      convex.setPoint(index++, p);
    }

    win.draw(circle);
    win.draw(rect);
    win.draw(convex);
  }
}

void Application::render_landmarks(std::vector<Feature> const& observed_features)
{
  std::vector<bool> observed_landmarks(landmarks.size(), false);
  for (auto const& f : observed_features)
    observed_landmarks[static_cast<unsigned int>(f.id)] = true;
  for (unsigned int i = 0; i < landmarks.size(); ++i)
  {
    sf::CircleShape circle(LANDMARK_SIZE);
    int m = LANDMARK_SIZE;
    circle.setPosition(landmarks[i]->x - m, landmarks[i]->y - m);
    if (observed_landmarks[i]) {
      circle.setFillColor(LANDMARK_OBSERVED_COLOR);
    } else {
      circle.setFillColor(LANDMARK_COLOR);
    }
    win.draw(circle);
  }
}

void Application::render_estimated_robot(Eigen::Vector3f const& pose)
{
  int m = ROBOT_SIZE;
  sf::CircleShape circle(ROBOT_SIZE);
  circle.setFillColor(sf::Color::Transparent);
  circle.setOutlineThickness(ROBOT_THICKNESS);
  circle.setOutlineColor(ROBOT_ESTIMATED_COLOR);
  circle.setPosition(pose.x() - m, pose.y() - m);

  sf::RectangleShape rect(sf::Vector2f(ROBOT_SIZE * 2, ROBOT_THICKNESS));
  rect.setRotation(TO_DEGREES(pose(2)));
  rect.setFillColor(ROBOT_ESTIMATED_COLOR);
  rect.setPosition(pose.x(), pose.y());

  win.draw(circle);
  win.draw(rect);
}

void Application::render_estimated_landmarks(const Eigen::MatrixXf &landmarks_est)
{
  for (int i = 0; i < landmarks_est.rows() / 2; ++i)
  {
    Eigen::Vector2f l = landmarks_est.block<2, 1>(i * 2, 0);
    if (l.x() > 0 && l.y() > 0)
    {
      sf::CircleShape circle(5);
      int m = 5;
      circle.setPosition(l.x() - m, l.y() - m);
      circle.setFillColor(sf::Color::Transparent);
      circle.setOutlineThickness(2);
      circle.setOutlineColor(sf::Color(150, 150, 150));
      win.draw(circle);
    }
  }
}

void Application::render_robot_ellipse(const EKF &ekf)
{
  auto ellipse = ellipse_from_cov(ekf.get_robot_cov());
  Eigen::Vector3f state = ekf.estimated_pose();

  draw_ellipse(state.head(2),ellipse, ROBOT_ELLIPSE_COLOR);
}

void Application::render_landmarks_ellipse(const EKF &ekf)
{
  for (int i = 0; i < static_cast<int>(landmarks.size()); ++i)
  {
    Eigen::Matrix2f cov = ekf.get_landmark_cov(i);
    if (cov(0, 0) < COV_INF)
    {
      auto ellipse(ellipse_from_cov(cov));
      Eigen::Vector2f position = ekf.estimated_landmark(i);
      draw_ellipse(position, ellipse, LANDMAKR_ELLIPSE_COLOR);
    }
  }
}

std::tuple<float, float, float> Application::ellipse_from_cov(const Eigen::Matrix2f &cov)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
  float e0 = std::sqrt(solver.eigenvalues()(0));
  float e1 = std::sqrt(solver.eigenvalues()(1));

  if (e0 > e1)
  {
    float theta = std::atan2(solver.eigenvectors()(1, 0),
                             solver.eigenvectors()(0, 0));
    return std::make_tuple(e0, e1, theta);
  }
  else
  {
    float theta = std::atan2(solver.eigenvectors()(1, 1),
                             solver.eigenvectors()(0, 1));
    return std::make_tuple(e1, e0, theta);
  }
}

void Application::draw_ellipse(const Eigen::Vector2f &position, std::tuple<float, float, float> ellipse, sf::Color const& color)
{
  sf::ConvexShape convex;
  convex.setFillColor(sf::Color::Transparent);
  convex.setOutlineThickness(ELLIPSE_THICKNESS);
  convex.setOutlineColor(color);
  size_t nb_points = ELLIPSE_NB_POINTS;
  convex.setPointCount(nb_points);
  float a = std::get<0>(ellipse) * ELLIPSE_SCALE;
  float b = std::get<1>(ellipse) * ELLIPSE_SCALE;
  float theta = std::get<2>(ellipse);

  float step = TO_RADIANS(360.0f) / (nb_points - 1);
  float angle = 0;
  for (unsigned int i = 0; i < nb_points; ++i)
  {
    sf::Vector2f p(std::cos(angle) * a,
                   std::sin(angle) * b);
    convex.setPoint(i, p);
    angle += step;
  }
  convex.setRotation(TO_DEGREES(theta));
  convex.setPosition(position.x(), position.y());

  // draw the axes
  sf::RectangleShape rect0(sf::Vector2f(a, ELLIPSE_THICKNESS));
  rect0.setRotation(TO_DEGREES(theta));
  rect0.setFillColor(color);
  rect0.setPosition(position.x(), position.y());

  sf::RectangleShape rect1(sf::Vector2f(a, ELLIPSE_THICKNESS));
  rect1.setRotation(TO_DEGREES(theta) + 180);
  rect1.setFillColor(color);
  rect1.setPosition(position.x(), position.y());

  sf::RectangleShape rect2(sf::Vector2f(b, ELLIPSE_THICKNESS));
  rect2.setRotation(TO_DEGREES(theta) + 90);
  rect2.setFillColor(color);
  rect2.setPosition(position.x(), position.y());

  sf::RectangleShape rect3(sf::Vector2f(b, ELLIPSE_THICKNESS));
  rect3.setRotation(TO_DEGREES(theta) - 90);
  rect3.setFillColor(color);
  rect3.setPosition(position.x(), position.y());

  win.draw(convex);
  win.draw(rect0);
  win.draw(rect1);
  win.draw(rect2);
  win.draw(rect3);
}
