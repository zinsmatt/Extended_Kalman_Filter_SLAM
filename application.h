#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>

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
  void render_landmarks();


private:
  sf::RenderWindow win;
  unsigned int width, height;
  std::string name;

  std::unique_ptr<Robot> robot;
  std::vector< std::unique_ptr<Landmark> > landmarks;

  // Grapphics parameters
  const int LANDMARKS_SIZE = 10;
  const int ROBOT_SIZE = 10;
  const int ROBOT_THICKNESS = 2;
  const sf::Color BACKGROUND_COLOR = sf::Color::Black;
  const sf::Color ROBOT_COLOR = sf::Color::Green;

  // Controls parameters
  const float ROBOT_ROTATION_STEP = 0.05f;   // in degrees
  const float ROBOT_MOVE_STEP = 2.5f;       // in world units (correspond to pixel)
};

#endif // WINDOW_H
