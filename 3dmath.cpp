#include "3dmath.h"

// Function to convert degrees to radians
float degrees_to_radians(float degrees) {
    return degrees * (PI / 180.0f);
}

// Function to convert radians to degrees
float radians_to_degrees(float radians) {
    return radians * (180.0f / PI);
}

mat4 quat::toMatrix() const{
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    return {
        1 - 2 * (yy + zz),  2 * (xy - wz),      2 * (xz + wy),      0,
        2 * (xy + wz),      1 - 2 * (xx + zz),  2 * (yz - wx),      0,
        2 * (xz - wy),      2 * (yz + wx),      1 - 2 * (xx + yy),  0,
        0,                  0,                  0,                  1
    };
}