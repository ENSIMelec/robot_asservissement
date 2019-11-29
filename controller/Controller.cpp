//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
using namespace std;

Controller::Controller(ICodeurManager& codeurs, MoteurManager& motor): m_odometry(codeurs), m_motor(motor)
{
    // Init PID Controllers

    m_maxTranslationSpeed = 40; // mm/s
    m_maxRotationSpeed = M_PI; // rad/s
    m_maxPWM = 50; // -50 PWM

    // Speed PWM Controller
    //m_leftSpeedPID = PID(1.4, 0.005, 0,-m_maxPWM, m_maxPWM);
    //m_rightSpeedPID = PID(1.4, 0.005, 0,-m_maxPWM, m_maxPWM);

    // Translation Controller
     = PID(1,0,0,-m_maxTranslationSpeed,m_maxTranslationSpeed);
    m_rotationPID = PID(1,0,0,-m_maxRotationSpeed, m_maxRotationSpeed);

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
    float speedTranslation = m_translationPID.compute(m_odometry.getTotalDistance(), m_target.distance);
//    // un moumvement de rotation
//    float speedRotation = m_rotationPID.compute(m_odometry.getDeltaOrientation(),m_target.angle);

    speedTranslation = max(-m_maxTranslationSpeed, min(m_maxTranslationSpeed, speedTranslation));
//    speedRotation = max(-m_maxRotationSpeed, min(m_maxRotationSpeed, speedRotation));

//    int32_t leftPWM = speedTranslation - speedRotation;
//    int32_t rightPWM = speedTranslation + speedRotation;

    int32_t leftPWM = speedTranslation;
    int32_t rightPWM = speedTranslation;

    m_motor.setConsigne(leftPWM, rightPWM);


    // debug:
    cout << "[PID DISTANCE] Speed Translation : " << speedTranslation;
    //cout << "[PID ANGLE] Speed Rotation : " << speedRotation;
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
    m_target.distance = sqrt(pow(xError,2) + pow(yError,2));
    // orientation qui doit prendre le robot pour atteindre le point
    // m_target.angle = atan2(dy,dx) - T0
    m_target.angle = atan2f(yError, xError) - deltaPos.theta;

    // Borner la consigne Angle entre Pi et -Pi
    m_target.angle = MathUtils::inrange(m_target.angle, -M_PI, M_PI);

    // Direction (cap inférieur à -pi/2 et supérieur à pi/2)
    //gestion de la marche arrière si on dépasse point de consigne
    if(fabs(m_target.angle) < M_PI_2) {
        m_direction = Direction::FORWARD;
    }
    else {
        m_direction = Direction::BACKWARD;

        m_target.distance = (-1)*m_target.distance;
        m_target.angle += M_PI;
        // Borner la consigne Angle entre Pi et -Pi
        m_target.angle = MathUtils::inrange(m_target.angle, -M_PI, M_PI);
    }

    //debug:

    cout << "[CONSIGNE] TARGET ANGLE (°): " << MathUtils::rad2deg(m_target.angle) << " TARGET DISTANCE (mm) : " << m_target.distance << endl;
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
    float speedTranslation = m_translationPID.compute(m_odometry.getTotalDistance(),m_target.distance);

    // un moumvement de rotation
    float speedRotation = m_rotationPID.compute(m_odometry.getDeltaAngle(),m_target.angle);

    cout << "PID speedTranslation: " << speedTranslation << " | PID rotatationTransalation: " << speedRotation << endl;


    speedTranslation = max(-m_maxTranslationSpeed, min(m_maxTranslationSpeed, speedTranslation));
    speedRotation = max(-m_maxRotationSpeed, min(m_maxRotationSpeed, speedRotation));


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

    m_rotationPID.setGoal(m_target.angle);

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
    m_translationPID.setGoal(m_target.distance + m_odometry.getDeltaDistance());

}