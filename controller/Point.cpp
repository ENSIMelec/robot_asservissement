//
// Created by Taoufik on 20/12/2019.
//

#include "Point.h"

Point::Point(float x, float y, float theta, Controller::Trajectory mTrajectory) : m_x(x), m_y(y), m_theta(theta),
                                                                                     m_trajectory(mTrajectory) {

}

float Point::getMX() const {
    return m_x;
}

float Point::getMY() const {
    return m_y;
}

float Point::getMTheta() const {
    return m_theta;
}
