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


class Controller {

public:
    /**
     * Initialisation de l'asservisement
     * @param codeurs
     * @param motor
    */
    Controller(FakeCodeur& codeurs, MoteurManager& motor);
    void update();

    /**
     * @brief Consigne de de déplacement
     * Calcul erreur entre la consigne et la position actuelle du robot
    **/
    void targetCalcul();
    void updateSpeed();
    /**
     * Déplacement x, y, angle
     * Target Position
     *
     * @param x
     * @param y
     * @param angle
    */
    void gotoPoint(int x, int y, int angle);
    void translate();
    void rotate();
    void stop();

    /** enum Direction
	 *  \brief Sens de déplacement pour le robot.
	 */
    enum Direction {
        FORWARD=1 ///< Le robot avance en marche avant.
        , BACKWARD=-1 ///< Le robot avance en marche arrière.
    };

private:
    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_translationPID;
    PID m_rotationPID;

    // Target position
    Position m_targetPos;

    // Target distance et angle
    float m_targetDistance; // mm
    float m_targetAngle; // rad
    int m_direction;

    //Odometry
    Odometry m_odometry;

    // MoteurManager
    MoteurManager m_motor;

};


#endif //ROBOT_CONTROLLER_H
