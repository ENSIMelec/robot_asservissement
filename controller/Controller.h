//
// Created by Taoufik on 12/11/2019.
//

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H


#include "PID.h"
#include "../ICodeurManager.h";
#include "Odometry.h";

class Controller {

public:
    Controller(ICodeurManager& codeurs);
    void process();
    void targetCalcul();
    void gotoPoint(int x, int y, int angle);
    void translate(float amount);
    void rotate(float angle);

private:
    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_translationPID;
    PID m_rotationPID;

    // Target point
    float m_targetX;
    float m_targetY;

    // Target distance et angle
    float m_targetDistance;
    float m_targetAngle;

    //Odometry
    Odometry m_odometry;




};


#endif //ROBOT_CONTROLLER_H
