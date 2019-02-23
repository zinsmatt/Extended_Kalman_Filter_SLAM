#include "application.h"

#include "landmark.h"

Application::Application(unsigned int w, unsigned int h, std::string const& str) :
  width(w), height(h), name(str)
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
    }
    win.clear(BACKGROUND_COLOR);

    render_landmarks();
    win.display();
  }
}

void Application::add_landmark(float x, float y)
{
  landmarks.push_back(std::make_unique<Landmark>(x, y));
}

void Application::render_landmarks()
{
  for (auto& l : landmarks)
  {
    sf::CircleShape circle(LANDMARKS_SIZE);
    circle.setPosition(l->x, l->y);
    win.draw(circle);
  }
}
