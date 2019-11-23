#ifndef ROBOT_ODOMETRY_H
#define ROBOT_ODOMETRY_H

#include "ICodeurManager.h"
#include "cmath"
#include "iostream"
#include "MathUtils.h"

/**
 * @brief Structure de position.
 * \author Taoufik Tribki
 * Position est une structure de Odometry.h qui permet en une variable d'obtenir la totalité des informations à propos du positionnement du robot.
 */
struct Position
{
    /**
     * @brief Constructeur de Position.
     * Constructeur de Position qui initialise la position au coordonnées (0,0) et à l'angle 0.
     */
    Position() : x(0), y(0), theta(0){}

    /**
     * @brief  Constructeur de Position.
     *
     * Constructeur de Position qui initialise la position au coordonnées indiqués.
     *
     * @param x coordoonée en x initial.
     * @param y coordoonée en y initial.
     * @param theta angle initial.
     */
    Position(float x, float y, float theta) : x(x), y(y), theta(theta){}
    // Coordonnée en x
    float x;
    // Cordonnée en y
    float y;
    // Angle
    float theta;
};
/**
 * Calcul la position en temps réél du robot
 */
class Odometry {

public:

    explicit Odometry(ICodeurManager &codeurs);

    float getLinVel() const {return m_linVel;}
    float getAngVel() const {return m_angVel;}
    int getLastTime() const { return m_lastTime; }
    void debug();

    float getTotalTicksL() const;
    float getTotalTicksR() const;

    /**
     * @brief Retourne la position
     * @return
     */
    const Position &getPosition() const { return m_pos; }
    void setPosition(float x, float y, float theta){ m_pos.x = x; m_pos.y = y; m_pos.theta = theta;}

    /**
    * @brief Calcule la nouvelle position et la nouvelle vitesse.
    * détermine la nouvelle vitesse instantanée et la nouvelle position.
    */
    void update();

    float getDeltaDistance() const;
    float getDeltaAngle() const;
    float getDeltaOrientation() const;
    float getTotalDistance() const { return m_distance; }

protected:

    Position m_pos; /*!< Structure de position de Odometry. */
    float m_linVel = 0; /*!< Vitesse lineaire en mm/s. */
    float m_angVel = 0; /*!< Vitesse angulaire en rad/s.*/

    /**
     * Total des tics
     */
    float m_totalTicksL = 0;
    float m_totalTicksR = 0;

    /**
     * Distance en mm parcouru entre un intervalle de temps
     */
    float m_dDistance = 0;

    float m_dAngle = 0;

    /**
     * Total distance parcouru (odometry test)
     */
     float m_distance = 0;

    /**
     * Angle en rad entre un intervalle de temps
     */
    float m_dOrientation = 0;

    /**
     * Time
     */
    int m_lastTime = 0;

    /**
     * Coefficient pour tics -> mm
     */
    float TICK_RIGHT_TO_MM = 0; /* Coefficient Longeur Gauche */
    float TICK_LEFT_TO_MM = 0; /* Coefficient Longeur Droit */

    /**
     * Coefficient pour tics -> rad
     */
    float TICK_RIGHT_TO_RAD = 0; /* Coefficient Angle Gauche */
    float TICK_LEFT_TO_RAD = 0; /* Coefficient Angle Droit */


    ICodeurManager& m_codeurs;

};


#endif //ROBOT_ODOMETRY_H
