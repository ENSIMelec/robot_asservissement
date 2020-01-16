//
// Created by Taoufik on 12/11/2019.
//

/**
 * TODO:
 * -Création de plusieurs points (stratégie)
 *  -Asservissement vitesse, ramp
 *  -Gestion du point d'arrivé (remove correction angle when we are in windowed point)
 *  - Add trajectory goto distance+angle
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

    this->m_maxPWM = 100;

    // Polar position PID
    this->m_distancePID = PID(
            m_config.getPIDkpDep(),
            m_config.getPIDkiDep(),
            m_config.getPIDkdDep(),
            -m_maxPWM,
            m_maxPWM
    );
    this->m_anglePID = PID(
            m_config.getPIDkpA(),
            m_config.getPIDkiA(),
            m_config.getPIDkdA(),
            -m_maxPWM,
            m_maxPWM
    );


    // Speed PID
    this->m_maxTranslationSpeed = 500; // mm/s
    this->m_maxRotationSpeed = 2*M_PI;

    this->m_leftSpeedPID = PID(1,0,0, 0, m_maxTranslationSpeed);
    this->m_rightSpeedPID = PID(1,0,0, 0, m_maxRotationSpeed);

    // Enable Control System
    this->m_controlDistance = true;
    this->m_controlAngle = true;
}
/**
 * CS Loop
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
    if(is_target_reached()) {
        // make_trajectory_stop();
        // motors_stop();

    }

}

/**
 * Effectuer un trajectoire en XY
 * update angle and/or distance
 * @param x_voulu en mm
 * @param y_voulu en mm
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
}
/**
 * Effectuer un trajectoire theta
 * @param angle_voulu  en rad
 */
void Controller::trajectory_theta(float angle_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // set consigne
    m_consign.distance = 0;
    m_consign.angle = deltaPos.theta - angle_voulu;
    // Borner l'angle
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

}
void Controller::trajectory_stop() {
    // set consigne angle et distance en 0
    m_consign.distance = 0;
    m_consign.angle = 0;
}

/**
 * Calcul PID
 * @param consigne_distance : distance to do in mm (is the distance between the robot and the goal position)
 * @param consigne_theta : angle to do in rad (is the angle to the goal relative to the heading of the robot)
 */
void Controller::update_speed(float consigne_distance, float consigne_theta) {

    cout << "CONSIGNE_DISTANCE " << consigne_distance << " - | CONSIGNE THETA:  " << MathUtils::rad2deg(consigne_theta) << endl;

    // un mouvement en distance
    // PID distance activé
    if(m_controlDistance) {
        m_speedDistance = m_distancePID.compute(m_odometry.getDeltaDistance(), consigne_distance);
        // borner la distance (pas nécessaire, la vitesse est déjà borner dans le PID)
        m_speedDistance = max(-m_maxPWM, min(m_maxPWM, m_speedDistance));
    }

    // un mouvement de rotation
    // PID angle activé
    if(m_controlAngle) {
        m_speedAngle = m_anglePID.compute(m_odometry.getDeltaTheta(), consigne_theta);
        // Borner (pas nécessaire, la vitesse est déjà borner dans le PID)
        m_speedAngle = max(-m_maxPWM, min(m_maxPWM, m_speedAngle));
    }


    int leftPWM = m_speedDistance + m_speedAngle;
    int rightPWM = m_speedDistance - m_speedAngle;

    m_motor.setConsigne(leftPWM, rightPWM);


    // debug:
    cout << "[ERROR DISTANCE] Error distance : " << m_distancePID.getError() << endl;
    cout << "[ERROR ANGLE] Error Angle : " << m_anglePID.getError() << endl;
    cout << "[PID DISTANCE] Speed Translation : " << m_speedDistance;
    cout << "[PID ANGLE] Speed Rotation : " << m_speedAngle;
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
    m_distancePID.resetErrors();
    m_anglePID.resetErrors();
}
/** near the target (dist in mm and angle in rad) **/
bool Controller::is_target_reached() {

    int distance_tolerance = 5; // mm
    float angle_tolerance = MathUtils::deg2rad(1);

    // get errors from PID
    return abs(m_distancePID.getError()) < distance_tolerance
           && (abs(m_anglePID.getError()) < angle_tolerance);
}
/** near the target (dist in x,y) ? */
bool Controller::is_target_reached_xy() {

    float distance_tolerance = 10; // mm

    // target position
    float x1 = m_targetPos.x;
    float y1 = m_targetPos.y;

    // actual position
    float x2 = m_odometry.getPosition().x;
    float y2 = m_odometry.getPosition().y;

    return  (sqrt(pow((x2 - x1),2) + pow((y2 - y1), 2)) < distance_tolerance);

}
/* near the angle target in radian ? */
bool Controller::is_target_reached_angle() {

    float angle_tolerance = MathUtils::deg2rad(1);

    return  (m_consign.angle < angle_tolerance);
}


//void Controller::set_consigne_distance_theta(float new_distance, float new_angle) {
//
//    m_consign.distance = new_distance  + m_odometry.getDeltaDistance();
//    m_consign.angle    = new_angle     + m_odometry.getDeltaOrientation();
//}

float Controller::quadramp_filter() {

    // const
    float afrein = 200;
    float vmax = 50;
    float amax = 60;
    float dt = m_odometry.getLastTime();
    float vdist = 0;
    float distance_now = m_consign.distance;

    float vrob = m_odometry.getDeltaDistance() / m_odometry.getLastTime();

    //pivot
    float dfrein = (pow(vrob,2)  / 2 * afrein);

    // Decceleration  : the pivot has been crossed, so we reverse the acceleration.
    // As the speed decreases, the pivot decreases. In fact we keep the pivot nearby
    if(distance_now < dfrein) {
        vdist = vrob - (dt * afrein);
    }
    // acceleration  : the goal is far away, so we accelerate
    else if (vrob < vmax) {
        vdist = vrob + (dt * amax);
    }
    // constant speed : the maximal speed is atteined, so the acceleration is ignored
    // (this phase may not exist if the position is not far away)
    else {
        vdist = vmax;
    }

    // algorithm must shut down on window

    return vdist;
}
/** return true if traj is nearly finished */
bool Controller::is_trajectory_reached() {

    switch (m_trajectory) {
        case THETA:
            return this->is_target_reached_angle();
        case XY_ABSOLU:
            return this->is_target_reached_xy();
        case LOCKED:
            return true;
        case NOTHING:
            return true;
        default:
            return false;
    }

}
