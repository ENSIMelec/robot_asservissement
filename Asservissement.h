//
// Created by Taoufik on 11/11/2019.
//

#ifndef ROBOT_ASSERVISSEMENT_H
#define ROBOT_ASSERVISSEMENT_H

#include <iostream>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "ICodeurManager.h"
#include "MoteurManager.h"


class Asservissement {


public:
    Asservissement(MoteurManager& moteurs, ICodeurManager& codeurs);


};


#endif //ROBOT_ASSERVISSEMENT_H
