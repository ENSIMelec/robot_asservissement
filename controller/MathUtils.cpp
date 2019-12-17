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
float MathUtils::simple_modulo_2pi(float a)
{
    if (a < -M_PI) {
        a += (2*M_PI);
    }
    else if (a > M_PI) {
        a -= (2*M_PI);
    }
    return a;
}

/** do a modulo 2.pi -> [-Pi,+Pi] */
float MathUtils::modulo_2pi(float a)
{
    double res = a - (((int32_t) (a/(2*M_PI))) * (2*M_PI));
    return simple_modulo_2pi(res);
}