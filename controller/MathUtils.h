//
// Created by Taoufik on 16/11/2019.
//

#ifndef ROBOT_MATHUTILS_H
#define ROBOT_MATHUTILS_H

#include "cmath"


class MathUtils final {

public:

    static float periodicmod(float x, float y);
    static float inrange(float x, float min, float max);
    static float inline deg2rad(float deg);
    static float inline rad2deg(float rad);
    static float inline micros2sec(float rad);
};


#endif //ROBOT_MATHUTILS_H
