#include "basketbuddy/drive.hpp"

// differential drive methods

// forward kinematics
void forward_kinematics(const int v_x, const int omega, float &left, float &right)
{
    // convert to m/s and rad/s
    auto linear = LINEAR_MAX * v_x / 100.0;
    auto angular = ANGULAR_MAX * omega / 100.0;
    left = (linear - HALF_AXLE * angular) / WHEEL_RADIUS;
    right = (linear + HALF_AXLE * angular) / WHEEL_RADIUS;
}

// inverse kinematics
void inverse_kinematics(const int left, const int right, int &v_x, int &omega)
{
    v_x = (left + right) * WHEEL_RADIUS / 2;
    omega = (right - left) * WHEEL_RADIUS / AXLE_LENGTH;
}
