#ifndef CONFIG_H
#define CONFIG_H

#define TO_DEGREES(x) x * 57.2957795131f
#define TO_RADIANS(x) x * 0.01745329251f
#define PI 3.14159265359f

const float ROBOT_FOV = TO_RADIANS(60);
const float ROBOT_RANGE_VIEW = 150;

const float SENSOR_RANGE_NOISE_STDDEV = 10.0f;
const float SENSOR_BEARING_NOISE_STDDEV = 0.1f;

#endif // CONFIG_H
