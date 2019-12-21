//
// Created by Taoufik on 12/11/2019.
//

/**
 * TODO:
 *  -Test asservissement angle+distance
 *  -Gestion du point d'arrivé
 *  - Add trajectory goto distance+angle
 *  -Création de plusieurs points (stratgie)
 *  -Rajouter des coefs pour corriger les moteurs en ligne droite sans asservissement angle
 *  -Création du module pour détection lidar
 *  -Lidar path planning
 *
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
 * Asservissement boucle
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
            trajectory_theta(m_targetPos.theta);
            break;
        case XY_ABSOLU:
            trajectory_xy(m_targetPos.x, m_targetPos.y);
            break;
        case LOCKED:
            trajectory_stop();
            break;

        case NOTHING:
            //il n'y a plus rien a faire
            break;

    }

    cout << "[CONSIGNE] TARGET ANGLE (°): " << MathUtils::rad2deg(m_consign.angle) << endl;
    cout << "[CONSIGNE] TARGET DISTANCE (mm) : " << m_consign.distance << endl;
    cout << "[CONSIGNE] DIRECTION: " << m_direction << endl;


    // envoye des consignes au PID
    update_speed(m_consign.distance, m_consign.angle);

    // gestion d'arrivé
    if(position_reached()) {
        // make_trajectory_stop();
        // motors_stop();

    }

}

/**
 * Effectuer un trajectoire en XY
 * update angle and/or distance
 * @param x_voulu
 * @param y_voulu
 */
void Controller::trajectory_xy(float x_voulu, float y_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // calcul des erreurs ( prendre la position du robot comme zero)
    float x_diff = (x_voulu - deltaPos.x);
    float y_diff = (y_voulu - deltaPos.y);

    // coordonnées cart -> polaire
    // distance entre la position du robot à instant t, et son objectif
    m_consign.distance = sqrt(x_diff * x_diff + y_diff * y_diff);

    // orientation qui doit prendre le robot pour atteindre le point
    m_consign.angle = atan2f(y_diff, x_diff) - deltaPos.theta;

    // Borner la consigne Angle entre [-pi, pi]
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

    // Direction (cap inférieur à -pi/2 et supérieur à pi/2)
    //gestion de la marche arrière si on dépasse point de consigne
    // TODO: à tester ( cas < -90  pas nécessaire)
    if (m_consign.angle > M_PI_2)
    {
        m_direction = Direction::BACKWARD;
        m_consign.angle -= M_PI;
        m_consign.distance = -m_consign.distance;
    }
    else if (m_consign.angle < -M_PI_2)
    {
        m_direction = Direction::BACKWARD;
        m_consign.angle += M_PI;
        m_consign.distance = -m_consign.distance;
    }
    else {
        m_direction = Direction::FORWARD;
    }
    // Borner l'angle [-pi, pi]
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

    cout << "[CONSIGNE] X_DIFF = " << x_diff << " | Y_DIFF = " << y_diff  << endl;
    //m_trajectory = null;
}
/**
 * Effectuer un trajectoire theta
 * @param angle_voulu  en deg
 */
void Controller::trajectory_theta(float angle_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // set consigne
    m_consign.distance = 0;
    m_consign.angle = angle_voulu - deltaPos.theta;
    // Borner l'angle
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

}
void Controller::trajectory_stop() {
    // set consigne angle et distance en 0
    m_consign.distance = 0;
    m_consign.angle = 0;
    //set_consigne_distance_theta(0,0);
}

/**
 * Calcul PID
 * @param consigne_distance : distance to do in mm (is the distance between the robot and the goal position)
 * @param consigne_theta : angle to do in rad (is the angle to the goal relative to the heading of the robot)
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

    m_consign.distance = new_distance  + m_odometry.getDeltaDistance();
    m_consign.angle    = new_angle     + m_odometry.getDeltaOrientation();
}

float Controller::ramp_distance() {

    // const
    float afrein = 200;
    float vmax = 50;
    float amax = 60;
    float dt = m_odometry.getLastTime();
    float vdist = 0;
    float distance_now = m_consign.distance;

    float vrob = m_odometry.getDeltaDistance() / m_odometry.getLastTime();

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
/** return true if traj is nearly finished */
bool Controller::trajectory_reached() {

    int distance_tolerance = 10;
    float angle_tolerance = MathUtils::deg2rad(3);

    switch (m_trajectory) {

        case THETA:
            break;
        case XY_ABSOLU:

            break;
        case LOCKED:
            break;
        case NOTHING:
            break;
        default:
            return 0;
    }

}