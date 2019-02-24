#ifndef FEATURE_H
#define FEATURE_H


/**
 * @brief The Feature struct represent a detection made by the robot
 */
struct Feature
{
  float range;    // distance between the robot and the detection
  float bearing;  // angle between the robot orientation and the detection
  float s;        // the signature of the corresponding landmark
  int id;         // id to the corresponding landmark
};

#endif // FEATURE_H
