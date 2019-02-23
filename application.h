#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>

class Landmark;

class Application
{
public:
  Application(unsigned int w, unsigned int h, std::string const& str);
  ~Application();

  void exec();

  void add_landmark(float x, float y);

private:
  void render_landmarks();


private:
  sf::RenderWindow win;
  unsigned int width, height;
  std::string name;

  std::vector< std::unique_ptr<Landmark> > landmarks;

  const int LANDMARKS_SIZE = 10;
  const sf::Color BACKGROUND_COLOR = sf::Color::Black;
};

#endif // WINDOW_H
