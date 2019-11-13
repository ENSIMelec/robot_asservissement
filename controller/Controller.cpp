//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
#include "PID.h"
#include "Odometry.h"
#include "../MoteurManager.h"

using namespace std;
/**
 * Initialisation de l'asservisement
 * @param codeurs
 * @param motor
 */
Controller::Controller(ICodeurManager& codeurs, MoteurManager& motor): m_odometry(codeurs), m_motor(motor)
{
    // Init PID Controllers

    // Speed Controller
    m_leftSpeedPID = PID(1.65, 0.005, 50,0,500);
    m_rightSpeedPID = PID(1.35, 0.005, 50,0,500);

    // Translation Controller
    m_translationPID = PID(4.35,0.000001,0,0,500);
    m_rotationPID = PID(3.5,0.000001,0,0, 2*M_PI);

}
void Controller::update()
{
    //Mise à jour de la position/orientation et de la vitesse du robot (Odométrie)
    m_odometry.update();

    // Calcul de la distance et de l'angle à fair pour aller au prochain point
    targetCalcul();

    updateSpeed();

    int32_t leftPWM = m_leftSpeedPID.compute(m_odometry.getLinVel(), m_leftSpeedPID.getCurrentGoal(), m_odometry.getLastTime());
    int32_t rightPWM = m_rightSpeedPID.compute(m_odometry.getLinVel(), m_leftSpeedPID.getCurrentGoal(), m_odometry.getLastTime());

    m_motor.setConsigne(leftPWM, rightPWM);

    // Déplacement en fonction du type du point
    //translate();
    //rotate();
}
/*
 * Calcul de consigne de distance et vitesse
 *
 */
void Controller::targetCalcul()
{
    // récupérer la position actuelle du robot
    Position initialPos = m_odometry.getPosition();


    float dx = (m_targetPos.x - initialPos.x);
    float dy = (m_targetPos.y - initialPos.y );

    // consigne distance
    m_targetDistance = sqrt(pow(dx,2) + pow(dy,2));
    // consigne angle
    m_targetAngle = atan2f(dx, dy);

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
/**
 * Rotation du robot asservi
 * @TODO: add flag for current statut of robot
 * @param angle
 */
void Controller::rotate()
{
    // reset PID
    m_rotationPID.resetErrors();

    m_rotationPID.setGoal(m_targetAngle);

}
/**
 * Déplacement asservi
 * @param amount
 */
void Controller::translate()
{
    // reset PID
    m_translationPID.resetErrors();

    // préciser la consigne de distance
    m_translationPID.setGoal(m_targetDistance + m_odometry.getDeltaDistance());

}
/**
 * Stop
 */
void Controller::stop() {

    // arrêt des moteurs
    m_motor.stop();

    m_translationPID.resetErrors();
    m_rotationPID.resetErrors();
}

/**
 * Asser du moteur
 */
void Controller::updateSpeed() {

    // si y a un mouvement translation
    float speedTranslation = m_translationPID.compute(m_odometry.getDeltaDistance(),m_targetDistance, m_odometry.getLastTime());

    // si y a un moumvement de rotation
    float speedRotation = m_rotationPID.compute(m_odometry.getDeltaOrientation(),m_targetAngle, m_odometry.getLastTime());

    // min / max
    float maxTranslationSpeed = 150; // 500   mm /s
    float maxRotationSpeed = 2*M_PI;  /* rad/s */

    speedTranslation = max(-maxTranslationSpeed, min(maxTranslationSpeed, speedTranslation));
    speedRotation = max(-maxRotationSpeed, min(maxRotationSpeed, speedRotation));


    m_leftSpeedPID.setGoal(speedTranslation - speedRotation);
    m_rightSpeedPID.setGoal(speedTranslation + speedRotation);

}

