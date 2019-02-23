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

  const unsigned int FPS = 60;      // this should not be changed. Otherwise all the control parameters will need to be adjusted
  const float DT = 1.0f / FPS;
  // Graphics parameters
  const int LANDMARK_SIZE = 10;
  const sf::Color LANDMARK_COLOR = sf::Color::White;
  const sf::Color LANDMARK_OBSERVED_COLOR = sf::Color::Red;

  const int ROBOT_SIZE = 10;
  const int ROBOT_THICKNESS = 2;
  const int ROBOT_VIEW_THICKNESS = 2;
  const sf::Color ROBOT_COLOR = sf::Color::Green;
  const sf::Color ROBOT_VIEW_COLOR = sf::Color::Yellow;


  const sf::Color BACKGROUND_COLOR = sf::Color::Black;

  // Controls parameters
  const float ROBOT_ROTATION_STEP = 1.5f;   // in degrees
  const float ROBOT_MOVE_STEP = 100;        // in world units (correspond to pixel)
};

#endif // WINDOW_H
