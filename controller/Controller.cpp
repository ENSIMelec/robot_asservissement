//
// Created by Taoufik on 12/11/2019.
//

/**
 * Checklist à verifier:
 *  - inverser rotation (right-left) odometrie  ou dans le controller
 *  - correction pid en int < à revoir>
 */

#include "Controller.h"
using namespace std;

Controller::Controller(ICodeurManager& codeurs, MoteurManager& motor, Config& config):
        m_odometry(codeurs,config),m_motor(motor), m_config(config)
{
    // Init PID Controllers

    m_maxPWM = 50;
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
/**
 * Asservissement
 */
void Controller::update()
{
    //synchronisation à une fréquence régulière!!

    //Mise à jour de la position/orientation et de la vitesse du robot (Odométrie) (via les roues codeuses)
    m_odometry.update();
    m_odometry.debug();

    // update des consignes en fonction du trajectoire choisi

    switch (m_trajectory) {

        case THETA:
            make_trajectory_theta(m_targetPos.theta);
            break;
        case XY_ABSOLU:
            make_trajectory_xy(m_targetPos.x, m_targetPos.y);
            break;
        case LOCKED:
            make_trajectory_stop();
            break;

        case NOTHING:
            //il n'y a plus rien a faire
            break;

    }

    cout << "[CONSIGNE] TARGET ANGLE (°): " << MathUtils::rad2deg(m_consigne.angle) << endl;
    cout << "[CONSIGNE] TARGET DISTANCE (mm) : " << m_consigne.distance << endl;
    cout << "[CONSIGNE] DIRECTION: " << m_direction << endl;



    //m_consigne.distance = is the distance between the robot and the goal position
    //m_consigne.angle = is the angle to the goal relative to the heading of the robot
    // envoye des commandes au moteurs
    update_speed(m_consigne.distance, m_consigne.angle);

    // gestion d'arrivé
    if(position_reached()) {
        // make_trajectory_stop();
        // motors_stop();
    }

}

/**
 * Assevissement en angle et distance
 */
void Controller::make_trajectory_xy(float x_voulu, float y_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // calcul des erreurs ( prendre la position du robot comme zero)
    float x_diff = (x_voulu - deltaPos.x);
    float y_diff = (y_voulu - deltaPos.y);

    // coordonnées cart -> polaire
    // distance entre la position du robot à instant t, et son objectif
    m_consigne.distance = sqrt(x_diff * x_diff + y_diff * y_diff);

    // orientation qui doit prendre le robot pour atteindre le point
    m_consigne.angle = atan2f(y_diff, x_diff) - deltaPos.theta;

    // Borner la consigne Angle entre [-pi, pi]
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
    // Borner l'angle [-pi, pi]
    m_consigne.angle = MathUtils::inrange(m_consigne.angle, -M_PI, M_PI);

    cout << "[CONSIGNE] X_DIFF = " << x_diff << " | Y_DIFF = " << y_diff  << endl;
    //m_trajectory = null;
}
/**
 * Asservissement angle
 * Calcul de la consigne en angle
 */
void Controller::make_trajectory_theta(float angle_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // set consigne
    m_consigne.distance = 0;
    m_consigne.angle = angle_voulu - deltaPos.theta;
    // Borner l'angle
    m_consigne.angle = MathUtils::inrange(m_consigne.angle, -M_PI, M_PI);

}
void Controller::make_trajectory_stop() {
    // set consigne angle et distance en 0
    set_consigne_distance_theta(0,0);
}

/**
 * Calcul PID
 * @param consigne_distance
 * @param consigne_theta
 */
void Controller::update_speed(float consigne_distance, float consigne_theta) {

    cout << "CONSIGNE_DISTANCE " << consigne_distance << " - | CONSIGNE THETA:  " << MathUtils::rad2deg(consigne_theta) << endl;
    // un mouvement en distance

    int speedTranslation = m_translationPID.compute(m_odometry.getDeltaDistance(), consigne_distance);
    // borner la distance (pas nécessaire, la vitesse est déjà borner dans le PID)
    speedTranslation = max(-m_maxPWM, min(m_maxPWM, speedTranslation));

    // un mouvement de rotation

    int speedRotation = m_rotationPID.compute(m_odometry.getDeltaTheta(), consigne_theta);
    // Borner (pas nécessaire, la vitesse est déjà borner dans le PID)
    speedRotation = max(-m_maxPWM, min(m_maxPWM, speedRotation));


    int leftPWM = speedTranslation + speedRotation;
    int rightPWM = speedTranslation - speedRotation;

    m_motor.setConsigne(leftPWM, rightPWM);


    // debug:
    cout << "[ERROR DISTANCE] Error distance : " << m_translationPID.getError() << endl;
    cout << "[ERROR ANGLE] Error Angle : " << m_rotationPID.getError() << endl;
    cout << "[PID DISTANCE] Speed Translation : " << speedTranslation;
    cout << "[PID ANGLE] Speed Rotation : " << speedRotation;
    cout << "[PWM] LEFT : " << leftPWM << " RIGHT: " << rightPWM << endl;
    cout << " ======================== " << endl;

    //correction eventuelle des commandes

    //écretage (si trop forte acceleration/décélérantion)

    //on regarde si on est pas arrivé à bon port
}
/**
 * Mettre en place le point voulu à atteindre dans la table
 * @param x  en mm
 * @param y  en mm
 * @param angle  en degré
 */
void Controller::set_point(int x, int y, int angle) {
    m_targetPos.x = x;
    m_targetPos.y = y;
    m_targetPos.theta = MathUtils::deg2rad(angle);

    // init distance qui reste
    distance_now = sqrt(pow(x,2) + pow(y,2));

}
/**
 * Stop des moteurs et réinitilisation des PID
 */
void Controller::motors_stop() {

    // arrêt des moteurs
    m_motor.stop();
    // reset des erreurs
    m_translationPID.resetErrors();
    m_rotationPID.resetErrors();
}

bool Controller::position_reached() {

    int distance_tolerance = 10; // mm
    float angle_tolerance = MathUtils::deg2rad(3);

    // get errors from PID
    return abs(m_translationPID.getError()) < distance_tolerance
           && (abs(m_rotationPID.getError()) < angle_tolerance);
}
void Controller::set_consigne_distance_theta(float new_distance, float new_angle) {

    m_consigne.distance = new_distance  + m_odometry.getDeltaDistance();
    m_consigne.angle    = new_angle     + m_odometry.getDeltaOrientation();
}

float Controller::ramp_distance() {

    // const
    float afrein = 200;
    float vmax = 50;
    float amax = 60;
    float dt = m_odometry.getLastTime();


    float distance_before = distance_now;
    distance_now = m_consigne.distance;
    float vrob = (distance_now - distance_before)/dt;

    float dfrein = (pow(vrob,2)  / 2 * afrein);

    if(distance_now < dfrein) {
        vdist = vrob - (dt * afrein);
    }
    else if (vrob < vmax) {
        vdist = vrob + (dt * amax);
    }
    else {
        vdist = vmax;
    }

    return vdist;
}
