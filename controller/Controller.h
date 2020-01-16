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
    void set_point(int x, int y, int angle);
    void set_trajectory(Trajectory trajectory) { m_trajectory = trajectory; }
    void motors_stop();
    bool is_target_reached();
    bool is_target_reached_xy();
    bool is_target_reached_angle();


private:

    void update_speed(float consigne_distance, float consigne_theta);
    void trajectory_theta(float angle_voulu);
    void trajectory_xy(float x_voulu, float y_voulu);
    void trajectory_stop();
    float quadramp_filter();
    bool is_trajectory_reached();
    void trajectory_distance_finished();
    void trajectory_angle_finished();
    void trajectory_distance_angle();
    void set_consigne_distance_theta(float new_distance, float new_angle);

    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_distancePID;
    PID m_anglePID;

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

    } m_consign;

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

    // enabled cs system
    bool m_controlDistance = true;
    bool m_controlAngle = true;

    // cs speed
    int m_speedDistance = 0;
    int m_speedAngle = 0;

};


#endif //ROBOT_CONTROLLER_H
