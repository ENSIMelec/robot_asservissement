//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
using namespace std;

Controller::Controller(ICodeurManager& codeurs, MoteurManager& motor, Config& config):
m_odometry(codeurs), m_motor(motor), m_config(config)
{
    // Init PID Controllers

    m_maxTranslationSpeed = 70; // mm/s
    m_maxRotationSpeed = M_PI; // rad/s

    m_maxPWM = 70; // -50 PWM

    // Speed PWM Controller
    //m_leftSpeedPID = PID(1.4, 0.005, 0,-m_maxPWM, m_maxPWM);
    //m_rightSpeedPID = PID(1.4, 0.005, 0,-m_maxPWM, m_maxPWM);

    // Translation Controller

    m_translationPID = PID(
            m_config.getPIDkpDep(),
            m_config.getPIDkiDep(),
            m_config.getPIDkdDep(),
            -m_maxPWM,
            m_maxPWM
     );

    m_rotationPID = PID(
            m_config.getPIDkpA(),
            m_config.getPIDkiA(),
            m_config.getPIDkdA(),
            -m_maxPWM,
            m_maxPWM
    );

}
void Controller::update()
{
    //synchronisation à une fréquence régulière!!

    //Mise à jour de la position/orientation et de la vitesse du robot (Odométrie)
    m_odometry.update();
    m_odometry.debug();

    // Calcul de la distance et de l'angle à fair pour aller au prochain point
    updateConsigne();

    //calcul des réponses provenant des PIDs

    // un mouvement translation
//    float speedTranslation = m_translationPID.compute(m_odometry.getDeltaDistance(), m_consigne.distance);
//    // un moumvement de rotation
   int speedRotation = m_rotationPID.compute(m_odometry.getDeltaOrientation(),m_consigne.angle);

   //   speedTranslation = max(-m_maxPWM, min(m_maxPWM, speedTranslation));
    speedRotation = max(-m_maxPWM, min(m_maxPWM, speedRotation));

//    int32_t leftPWM = speedTranslation - speedRotation;
//    int32_t rightPWM = speedTranslation + speedRotation;

    int leftPWM = -speedRotation;
    int rightPWM = speedRotation;

    m_motor.setConsigne(leftPWM, rightPWM);


    // debug:
    //cout << "[PID DISTANCE] Speed Translation : " << speedTranslation;
    cout << "[PID ANGLE] Speed Rotation : " << speedRotation;
    cout << "[PWM] LEFT : " << leftPWM << " RIGHT: " << rightPWM << endl;

    //correction eventuelle des commandes

    //écretage (si trop forte acceleration/décélérantion)

    //on regarde si on est pas arrivé à bon port


    // Déplacement en fonction du type du point
    //translate();
    //rotate();

    cout << " ======================== " << endl;
}
/**
 * Calcul de la consigne
 */
void Controller::updateConsigne()
{
    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // calcul des erreurs ( prendre la position du robot comme zero)
    float xError = (m_targetPos.x - deltaPos.x);
    float yError = (m_targetPos.y - deltaPos.y);

    // distance entre la position du robot à instant t, et son objectif (toujours positif? fait que avancer)
    m_consigne.distance = sqrt(pow(xError,2) + pow(yError,2));
    // orientation qui doit prendre le robot pour atteindre le point
    // m_consigne.angle = atan2(dy,dx) - T0
    //m_consigne.angle = atan2f(yError, xError) - deltaPos.theta;
    m_consigne.angle = atan2f(yError, xError) - deltaPos.theta;

    // Borner la consigne Angle entre Pi et -Pi
    m_consigne.angle = MathUtils::inrange(m_consigne.angle, -M_PI, M_PI);

    // Direction (cap inférieur à -pi/2 et supérieur à pi/2)
    //gestion de la marche arrière si on dépasse point de consigne
    if(fabs(m_consigne.angle) < M_PI_2) {
        m_direction = Direction::FORWARD;
    }
    else {
        m_direction = Direction::BACKWARD;

        m_consigne.distance = (-1)*m_consigne.distance;
        m_consigne.angle += M_PI;
        // Borner la consigne Angle entre Pi et -Pi
        m_consigne.angle = MathUtils::inrange(m_consigne.angle, -M_PI, M_PI);
    }


    cout << "[CONSIGNE] TARGET ANGLE (°): " << MathUtils::rad2deg(m_consigne.angle) << " TARGET DISTANCE (mm) : " << m_consigne.distance << endl;
    cout << "[CONSIGNE] DIRECTION: " << m_direction << endl;
}
/**
 * Mettre en place le point voulu à atteindre dans la table
 * @param x  en mm
 * @param y  en mm
 * @param angle  en degré
 */
void Controller::gotoPoint(int x, int y, int angle) {
    m_targetPos.x = x;
    m_targetPos.y = y;
    m_targetPos.theta = MathUtils::deg2rad(angle);
}
/**
 * Stop des moteurs et réinitilisation des PID
 */
void Controller::stop() {

    // arrêt des moteurs
    m_motor.stop();
    // reset des erreurs
    m_translationPID.resetErrors();
    m_rotationPID.resetErrors();
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

    m_rotationPID.setGoal(m_consigne.angle);

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
    m_translationPID.setGoal(m_consigne.distance + m_odometry.getDeltaDistance());

}

bool Controller::positionReached() {

    int M_PRECISION_DELTA = 10; // mm
    // get errors from PID
    return abs(m_translationPID.getError() < M_PRECISION_DELTA) != 0;
}

void Controller::updateSpeed() {

}
