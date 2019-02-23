#include <iostream>

#include "application.h"

using namespace std;

int main()
{
  Application app(800, 600, "EKF SLAM");
  app.exec();

  return 0;
}
