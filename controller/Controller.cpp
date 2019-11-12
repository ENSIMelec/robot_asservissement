//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
#include "PID.h"
#include "Odometry.h"
#include "../MoteurManager.h"

/**
 * Asservissement
 * @param codeurs
 */
Controller::Controller(ICodeurManager& codeurs): m_odometry(codeurs)
{
    // Init odometry

    // Init PID Controllers

    // Speed Controller
    m_leftSpeedPID = PID(1.65, 0.005, 50,0,500);
    m_rightSpeedPID = PID(1.35, 0.005, 50,0,500);

    // Translation Controller
    m_translationPID = PID(4.35,0.000001,0,0,500);
    m_rotationPID = PID(3.5,0.000001,0,0, 2*M_PI);
}
void Controller::process()
{
    //Mise à jour de la position et de la vitesse du robot (Odométrie)
    m_odometry.process();

    // Calcul de la distance et de l'angle à fair pour aller au prochain point
    targetCalcul();

    // Déplacement en fonction du type du point
}
/**
 * Rotation du robot
 * @param angle
 */
void Controller::rotate(float angle)
{
    // reset PID
    m_rotationPID.resetErrors();

    m_targetPos.theta = angle;

}
/**
 * Déplacement
 * @param amount
 */
void Controller::translate(float distance)
{
    // reset PID
    m_translationPID.resetErrors();

    // préciser la consigne de distance
    m_targetDistance = distance;



}
/*
 * Calcul de consigne de distance et vitesse
 *
 */
void Controller::targetCalcul()
{
    // Consigne Déplacement
    Position initialPos = m_odometry.getPosition();

    // Consigne Distance
    m_targetDistance = sqrt(pow(( m_targetPos.x - initialPos.x ),2) + pow((m_targetPos.y - initialPos.y ),2));

    // Consigne Angle
    m_targetPos.theta = atan2((m_targetPos.x - initialPos.x),(m_targetPos.y - initialPos.y))*180/M_PI;

}
/**
 * Déplacement x, y, angle
 *
 * @param x
 * @param y
 * @param angle
 */
void Controller::gotoPoint(int x, int y, int angle) {
    m_targetPos.x = x;
    m_targetPos.y = y;
    m_targetPos.theta = angle;
}
