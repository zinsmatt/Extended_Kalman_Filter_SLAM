#include "application.h"

#include "landmark.h"
#include "robot.h"
#include <iostream>

Application::Application(unsigned int w, unsigned int h, std::string const& str) :
  width(w), height(h), name(str), robot(nullptr)
{
  win.create(sf::VideoMode(width, height), name);

}

Application::~Application()
{
}

void Application::exec()
{
  while (win.isOpen())
  {
    sf::Event event;
    while (win.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        win.close();

      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
      {
        robot->orientation() -= ROBOT_ROTATION_STEP;
        std::cout << "left" << std::endl;
      }
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
      {
        robot->orientation() += ROBOT_ROTATION_STEP;
        std::cout << "right" << std::endl;
      }
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
      {
        robot->move(ROBOT_MOVE_STEP);
        std::cout << "up" << std::endl;
      }
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
      {
        robot->move(-ROBOT_MOVE_STEP);
      }
    }
    win.clear(BACKGROUND_COLOR);

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
