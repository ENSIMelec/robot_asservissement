//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
using namespace std;

Controller::Controller(ICodeurManager& codeurs, MoteurManager& motor, Config& config): m_odometry(codeurs,config), m_motor(motor), m_config(config)
{
    // Init PID Controllers

    m_maxTranslationSpeed = 70; // mm/s
    m_maxRotationSpeed = M_PI; // rad/s

    m_maxPWM = 50; // -50 PWM

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


    // update consigne en fonction du point pour aller au prochain

    // Calcul de la distance et de l'angle à fair pour aller au prochain point
    updateConsigne();

    //calcul des réponses provenant des PIDs

    // un mouvement en distance

    int speedTranslation = m_translationPID.compute(m_odometry.getDeltaDistance(), m_consigne.distance);
    // borner la distance (pas nécessaire, la vitesse est déjà borner dans le PID)
    speedTranslation = max(-m_maxPWM, min(m_maxPWM, speedTranslation));

   // un moumvement de rotation

   int speedRotation = m_rotationPID.compute(m_odometry.getDeltaOrientation(),m_consigne.angle);
   // Borner (pas nécessaire, la vitesse est déjà borner dans le PID)
   speedRotation = max(-m_maxPWM, min(m_maxPWM, speedRotation));


    //int leftPWM = speedTranslation - speedRotation;
    //int rightPWM = speedTranslation + speedRotation;

    int leftPWM = speedTranslation + speedRotation;
    int rightPWM = speedTranslation - speedRotation;

    m_motor.setConsigne(leftPWM, rightPWM);


    // debug:
    cout << "[PID DISTANCE] Speed Translation : " << speedTranslation;
    cout << "[PID ANGLE] Speed Rotation : " << speedRotation;
    cout << "[PWM] LEFT : " << leftPWM << " RIGHT: " << rightPWM << endl;
    cout << " ======================== " << endl;

    //correction eventuelle des commandes

    //écretage (si trop forte acceleration/décélérantion)

    //on regarde si on est pas arrivé à bon port


    // Déplacement en fonction du type du point

}
/**
 * Calcul de la consigne
 * m_consigne.distance = is the distance between the robot and the goal position
 * m_consigne.angle = is the angle to the goal relative to the heading of the robot
 */
void Controller::updateConsigne()
{
    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // calcul des erreurs ( prendre la position du robot comme zero)
    float x_diff = (m_targetPos.x - deltaPos.x);
    float y_diff = (m_targetPos.y - deltaPos.y);

    // coordonnées cart -> polaire
    // distance entre la position du robot à instant t, et son objectif
    m_consigne.distance = sqrt(x_diff * x_diff + y_diff * y_diff);

    // orientation qui doit prendre le robot pour atteindre le point
    m_consigne.angle = atan2f(y_diff, x_diff) - deltaPos.theta;

    // Borner la consigne Angle entre Pi et -Pi
    m_consigne.angle = MathUtils::inrange(m_consigne.angle, -M_PI, M_PI);

    // Direction (cap inférieur à -pi/2 et supérieur à pi/2)
    //gestion de la marche arrière si on dépasse point de consigne
    // TODO: à tester ( cas < -90  pas nécessaire)
    if (m_consigne.angle > M_PI_2)
    {
        m_direction = Direction::BACKWARD;
        m_consigne.angle -= M_PI;
        m_consigne.distance = -m_consigne.distance;
    }
    else if (m_consigne.angle < -M_PI_2)
    {
        m_direction = Direction::BACKWARD;
        m_consigne.angle += M_PI;
        m_consigne.distance = -m_consigne.distance;
    }
    else {
        m_direction = Direction::FORWARD;
    }
    // Borner l'angle
    m_consigne.angle = MathUtils::inrange(m_consigne.angle, -M_PI, M_PI);

    //TODO : gestion point non atteignable
    //(si l'on demande un point trop prés du robot et à la perpendiculaire de la direction du robot il se met à tourner autour du point)

    cout << "X_DIFF = " << x_diff << " | Y_DIFF = " << y_diff  << endl;
    cout << "[CONSIGNE] TARGET ANGLE (°): " << MathUtils::rad2deg(m_consigne.angle) << endl;
    cout <<" [CONSIGNE] TARGET DISTANCE (mm) : " << m_consigne.distance << endl;
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


bool Controller::positionReached() {

    int distance_tolerance = 10; // mm
    // get errors from PID
    return abs(m_translationPID.getError()) < distance_tolerance;
}

void Controller::updateSpeed() {

}
