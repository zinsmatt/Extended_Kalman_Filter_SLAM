#include <iostream>

#include "application.h"

using namespace std;

int main()
{
  Application app(800, 600, "EKF SLAM");

  app.add_landmark(100, 100);
  app.add_landmark(200, 100);
  app.add_landmark(100, 400);
  app.add_landmark(500, 500);

  app.exec();

  return 0;
}
