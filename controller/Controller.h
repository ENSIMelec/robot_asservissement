//
// Created by Taoufik on 12/11/2019.
//

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H


#include "PID.h"
#include "ICodeurManager.h"
#include "Odometry.h"
#include "MoteurManager.h"

class Controller {

public:
    Controller(FakeCodeur& codeurs, MoteurManager& motor);
    void update();
    void targetCalcul();
    void updateSpeed();
    void gotoPoint(int x, int y, int angle);
    void translate();
    void rotate();
    void stop();

private:
    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_translationPID;
    PID m_rotationPID;

    // Target position
    Position m_targetPos;

    // Target distance et angle
    float m_targetDistance; // mm
    float m_targetAngle; // rad

    //Odometry
    Odometry m_odometry;

    // MoteurManager
    MoteurManager m_motor;

};


#endif //ROBOT_CONTROLLER_H
