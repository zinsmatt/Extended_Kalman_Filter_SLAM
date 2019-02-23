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
  bool forward= false, left = false, right = false, backward = false;
  while (win.isOpen())
  {
    sf::Event event;
    while (win.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        win.close();
      if (event.type == sf::Event::KeyPressed)
      {
        if (event.key.code == sf::Keyboard::Left)
          left = true;
        if (event.key.code == sf::Keyboard::Right)
          right = true;
        if (event.key.code == sf::Keyboard::Up)
          forward = true;
        if (event.key.code == sf::Keyboard::Down)
          backward = true;
      }
      if (event.type == sf::Event::KeyReleased)
      {
        if (event.key.code == sf::Keyboard::Left)
          left = false;
        if (event.key.code == sf::Keyboard::Right)
          right = false;
        if (event.key.code == sf::Keyboard::Up)
          forward = false;
        if (event.key.code == sf::Keyboard::Down)
          backward = false;
      }
    }
    win.clear(BACKGROUND_COLOR);

    if (left)
      robot->orientation() -= ROBOT_ROTATION_STEP;
    if (right)
      robot->orientation() += ROBOT_ROTATION_STEP;
    if (forward)
      robot->move(ROBOT_MOVE_STEP);
    if (backward)
      robot->move(-ROBOT_MOVE_STEP);

    render_robot();
    render_landmarks();
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

void Application::render_landmarks()
{
  for (auto& l : landmarks)
  {
    sf::CircleShape circle(LANDMARKS_SIZE);
    int m = LANDMARKS_SIZE;
    circle.setPosition(l->x - m, l->y - m);
    win.draw(circle);
  }
}
