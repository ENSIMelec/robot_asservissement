//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
using namespace std;

Controller::Controller(FakeCodeur& codeurs, MoteurManager& motor): m_odometry(codeurs), m_motor(motor)
{
    // Init PID Controllers

    // Speed PWM Controller
    m_leftSpeedPID = PID(1, 0, 0,0,255);
    m_rightSpeedPID = PID(1, 0, 0,0,255);

    // Translation Controller
    m_translationPID = PID(1,0,0,0,500);
    m_rotationPID = PID(1,0,0,0, 2*M_PI);

}
void Controller::update()
{
    //Mise à jour de la position/orientation et de la vitesse du robot (Odométrie)
    m_odometry.update();
    m_odometry.debug();

    // Calcul de la distance et de l'angle à fair pour aller au prochain point
    targetCalcul();

    updateSpeed();

//    int32_t leftPWM = m_leftSpeedPID.compute(m_odometry.getLinVel(), m_leftSpeedPID.getCurrentGoal(), m_odometry.getLastTime());
//    int32_t rightPWM = m_rightSpeedPID.compute(m_odometry.getLinVel(), m_rightSpeedPID.getCurrentGoal(), m_odometry.getLastTime());


    int leftPWM = m_targetAngle + m_targetDistance;
    int rightPWM = m_targetAngle - m_targetDistance;

    cout << " leftPWM: " << leftPWM << " rightPWM: " << rightPWM << endl;
    m_motor.setConsigne(leftPWM, rightPWM);

    // Déplacement en fonction du type du point
    //translate();
    //rotate();

    cout << " ======================== " << endl;
}
/**
 * Calcul de la consigne
 */
void Controller::targetCalcul()
{
    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // calcul des erreurs ( prendre la position du robot comme zero)
    float xError = (m_targetPos.x - deltaPos.x);
    float yError = (m_targetPos.y - deltaPos.y);

    // distance entre la position du robot à instant t, et son objectif (toujours positif? fait que avancer)
    m_targetDistance = sqrt(pow(xError,2) + pow(yError,2));
    // orientation qui doit prendre le robot pour atteindre le point
    // m_targetAngle = atan2(dy,dx) - T0
    m_targetAngle = atan2f(yError, xError) - deltaPos.theta;

    // Borner la consigne Angle entre Pi et -Pi
    m_targetAngle = MathUtils::inrange(m_targetAngle, -M_PI, M_PI);

    // Direction
    if(fabs(m_targetAngle) < M_PI_2) {
        m_direction = Direction::FORWARD;
    }
    else {
        m_direction = Direction::BACKWARD;
    }

    cout << "[CONSIGNE] TARGET ANGLE (°): " << m_targetAngle * (180 / M_PI) << " TARGET DISTANCE (mm) : " << m_targetDistance << endl;
    cout << "[CONSIGNE] DIRECTION: " << m_direction << endl;

    // stratégie de mouvement
    // steps: Chargé la position et l'angle a atteindre!

}

void Controller::gotoPoint(int x, int y, int angle) {
    m_targetPos.x = x;
    m_targetPos.y = y;
    m_targetPos.theta = angle;
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

    // un mouvement translation
    float speedTranslation = m_translationPID.compute(m_odometry.getDeltaDistance(),m_targetDistance, m_odometry.getLastTime());

    // un moumvement de rotation
    float speedRotation = m_rotationPID.compute(m_odometry.getDeltaOrientation(),m_targetAngle, m_odometry.getLastTime());

    cout << "PID speedTranslation: " << speedTranslation << "PID rotatationTransalation: " << speedRotation << endl;
    // min / max
    float maxTranslationSpeed = 30; // 500   mm /s
    float maxRotationSpeed = 2*M_PI;  /* rad/s */

    speedTranslation = max(-maxTranslationSpeed, min(maxTranslationSpeed, speedTranslation));
    speedRotation = max(-maxRotationSpeed, min(maxRotationSpeed, speedRotation));


    cout << "Speed Translation : " << speedTranslation << " Speed Rotation : " << speedRotation << endl;

    m_leftSpeedPID.setGoal(speedTranslation - speedRotation);
    m_rightSpeedPID.setGoal(speedTranslation + speedRotation);

    cout << "left speed goal : " << m_leftSpeedPID.getCurrentGoal() << " left right goal : " << m_rightSpeedPID.getCurrentGoal() << endl;

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