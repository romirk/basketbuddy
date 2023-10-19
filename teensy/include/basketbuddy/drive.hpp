#pragma once

#define AXLE_LENGTH 0.46985  // m
#define WHEEL_RADIUS 0.0508  // m
#define LINEAR_MAX 1         // m/s
#define ANGULAR_MAX 2        // rad/s

constexpr auto HALF_AXLE = AXLE_LENGTH / 2;
constexpr auto V_MAX = LINEAR_MAX / WHEEL_RADIUS;

void forward_kinematics(const int v_x, const int omega, float &left,
                        float &right);
void inverse_kinematics(const int left, const int right, int &v_x, int &omega);