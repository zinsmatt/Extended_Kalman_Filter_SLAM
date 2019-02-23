#include "application.h"

Application::Application(int w, int h, std::string const& str) :
  width(w), height(h), name(str)
{
  win.create(sf::VideoMode(width, height), name);

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
  }
}
