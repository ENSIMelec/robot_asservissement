//
// Created by Taoufik on 20/12/2019.
//

#include "Point.h"

Point::Point(float x, float y, float theta, Controller::Trajectory mTrajectory) : m_x(x), m_y(y), m_theta(theta),
                                                                                     m_trajectory(mTrajectory) {

}

float Point::getX() const {
    return m_x;
}

float Point::getY() const {
    return m_y;
}

float Point::getTheta() const {
    return m_theta;
}

Controller::Trajectory Point::getTrajectory() const {
    return m_trajectory;
}
