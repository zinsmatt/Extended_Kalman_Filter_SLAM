#ifndef CONFIG_H
#define CONFIG_H

#define TO_DEGREES(x) x * 57.2957795131f
#define TO_RADIANS(x) x * 0.01745329251f
#define PI 3.14159265359f

const float ROBOT_FOV = TO_RADIANS(60);
const float ROBOT_RANGE_VIEW = 150;


// velocity_noise          = alpha1*|velocity| + alpha2*|angular_velocity|
// angular_velocity_noise  = alpha3*|velocity| + alpha4*|angular_velocity|
const float ALPHA1 = 0.3f;
const float ALPHA2 = 0.0f;
const float ALPHA3 = 0.001f;
const float ALPHA4 = 0.1f;

//const float COMMAND_NOISE_VELOCITY_STDDEV = 0.13f;
//const float COMMAND_NOISE_ANGULAR_VELOCITY_STDDEV = 0.082f;

const float SENSOR_RANGE_NOISE_STDDEV = 5.0f;
const float SENSOR_BEARING_NOISE_STDDEV = 0.05f;

// 99% confidence
const float ELLIPSE_SCALE = 3.0348f;

// Controls parameters
const float ROBOT_ROTATION_SPEED = 1.5f;   // in radians
const float ROBOT_MOVE_SPEED = 100;        // in world units (correspond to pixel)

const float COV_INF = 1e5f;
#endif // CONFIG_H
