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

float inline MathUtils::deg2rad(float x) {
    return x * (180 / M_PI);
}
float inline MathUtils::rad2deg(float x) {
    return x * (M_PI / 180);
}
float inline MathUtils::micros2sec(float x) {
    return x * 0.000001;
}