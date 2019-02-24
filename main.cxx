#include <iostream>

#include "application.h"

using namespace std;

int main()
{
  Application app(800, 600, "EKF SLAM");

  app.init_robot(400, 300, 0);

  app.add_landmark(100, 100);
  app.add_landmark(200, 100);
  app.add_landmark(100, 400);
  app.add_landmark(500, 500);
  app.add_landmark(300, 250);
  app.add_landmark(250, 100);
  app.add_landmark(200, 400);
  app.add_landmark(200, 350);

  app.exec();

  return 0;
}
