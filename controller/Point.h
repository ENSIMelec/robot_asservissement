//
// Created by Taoufik on 20/12/2019.
//

#ifndef ROBOT_POINT_H
#define ROBOT_POINT_H


#include "Controller.h"

class Point {

public:
    Point(float mX, float mY, float mTheta, Controller::Trajectory mTrajectory);

    float getX() const;

    float getY() const;

    float getTheta() const;

private:

    float m_x = 0;
    float m_y = 0;
    float m_theta = 0;
    Controller::Trajectory m_trajectory = Controller::Trajectory::THETA;

};


#endif //ROBOT_POINT_H
