#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Window.hpp>

class Application
{
public:
  Application(int w, int h, std::string const& str);

  void exec();

private:
  sf::Window win;
  int width, height;
  std::string name;
};

#endif // WINDOW_H
