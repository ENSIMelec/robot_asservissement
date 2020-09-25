//
// Created by Taoufik on 20/12/2019.
//

#ifndef ROBOT_POINT_H
#define ROBOT_POINT_H

using namespace std;
#include "Controller.h"

class Point {

public:
    Point(float mX, float mY, float mTheta, Controller::Trajectory mTrajectory);
    float getX() const;
    float getY() const;
    float getTheta() const;

    float getMDistanceTolerance() const;

    void setMDistanceTolerance(float mDistanceTolerance);

    float getMAngleTolerance() const;

    void setMAngleTolerance(float mAngleTolerance);

    int getMSpeed() const;

    void setMSpeed(int mSpeed);

    const string &getMAction() const;

    void setMAction(const string &mAction);

    int getMTimeout() const;

    void setMTimeout(int mTimeout);

    Controller::Trajectory getMTrajectory() const;

    void setMTrajectory(Controller::Trajectory mTrajectory);

    Controller::Trajectory getTrajectory() const;

private:

    float m_x = 0;
    float m_y = 0;
    float m_theta = 0;
    float m_distance_tolerance;
    float m_angle_tolerance;
    int m_speed;
    string m_action;
    int m_timeout;
    Controller::Trajectory m_trajectory = Controller::Trajectory::THETA;

};


#endif //ROBOT_POINT_H
