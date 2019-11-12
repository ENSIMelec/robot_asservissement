//
// Created by Taoufik on 12/11/2019.
//

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H


#include "PID.h"

class Controller {

public:
    Controller();
    void process();
    void translate(double amount);
    void rotate();

private:
    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_translationPID;
    PID m_rotationPID;

    // Target point
    double m_targetX;
    double m_targetY;

    // Target distance et angle
    double m_targetDistance;
    float m_targetAngle;


};


#endif //ROBOT_CONTROLLER_H
