//
// Created by Taoufik on 16/11/2019.
//

#include "MathUtils.h"

float MathUtils::inrange(float x, float min, float max) {
    return periodicmod(x - min, max - min) + min;
}

float MathUtils::periodicmod(float x, float y) {
    return fmod(fmod(x, y) + y, y);
}

float MathUtils::deg2rad(float deg) {
    return deg * (M_PI / 180);
}
float MathUtils::rad2deg(float rad) {
    return rad * (180 / M_PI);
}
float MathUtils::micros2sec(float sec) {
    return sec * 0.000001;
}