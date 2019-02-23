#include "application.h"

#include "landmark.h"
#include "robot.h"
#include <iostream>

Application::Application(unsigned int w, unsigned int h, std::string const& str) :
  width(w), height(h), name(str), robot(nullptr)
{
  win.create(sf::VideoMode(width, height), name);
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

    win.draw(circle);
    win.draw(rect);

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
