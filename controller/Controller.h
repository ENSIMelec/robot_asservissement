//
// Created by Taoufik on 12/11/2019.
//

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H


#include "PID.h"
#include "ICodeurManager.h"
#include "Odometry.h"
#include "MoteurManager.h"
#include "MathUtils.h"
#include "Config.h"


class Controller {

public:
    /**
     * Initialisation de l'asservisement
     * @param codeurs
     * @param motor
     * @param config
    */
    Controller(ICodeurManager& codeurs, MoteurManager& motor, Config& config);
    /**
     * Method asserv
     */
    void update();
    void update_speed(float consigne_distance, float consigne_theta);
    /**
     * Déplacement x, y, angle
     * Target Position
     *
     * @param x
     * @param y
     * @param angle
    */

    void set_point(int x, int y, int angle);
    void motors_stop();
    bool position_reached();

    /** enum Direction
	 *  \brief Sens de déplacement pour le robot.
	 */
    enum Direction {
        FORWARD     = 1, ///< Le robot avance en marche avant.
        BACKWARD    = -1 ///< Le robot avance en marche arrière.
    };

    enum Trajectory {
        THETA,
        XY_ABSOLU,
        LOCKED,
        NOTHING
    };
    void set_trajectory(Trajectory trajectory) { m_trajectory = trajectory; }
    void make_trajectory_theta(float angle_voulu);
    void make_trajectory_xy(float x_voulu, float y_voulu);
    void make_trajectory_stop();

private:
    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_translationPID;
    PID m_rotationPID;

    // Target position
    Position m_targetPos;

    //Trajectory actuelle
    Trajectory m_trajectory = NOTHING;

    /**
     * Structure de la consigne à atteindre
     */
    struct
    {
        float distance = 0.0; // mm
        float angle = 0.0; // rad

    } m_consigne;

    int m_direction = 0;

    float m_maxTranslationSpeed = 0; // mm/s
    float m_maxRotationSpeed = 0; // mm/s
    int m_maxPWM = 0;

    //Odometry
    Odometry m_odometry;

    // MoteurManager
    MoteurManager m_motor;
    //Config
    Config m_config;

};


#endif //ROBOT_CONTROLLER_H
