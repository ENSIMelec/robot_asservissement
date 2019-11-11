#include "Odometry.h"



Odometry::Odometry(ICodeurManager &codeurs) : codeurs(codeurs) {

    // Coefficient roue linéaire
    this->m_coeffLongL = 0.0056;
    this->m_coeffLongR = 0.0056;

    // Coefficient roue angle
    this->m_coeffAngL = 0.0065;
    this->m_coeffLongR = 0.0069;
}

/**
 * Odométrie
 * Calcul de la position et orientation du robot et vitesse entre deux intervalle de temps
 */

void Odometry::process() {

    // récupérer les tics des codeurs + réinitialisation
    codeurs.readAndReset();

    // récupérer les tics codeurs
    float ticksLeft = codeurs.getLeftTicks();
    float tickRight = codeurs.getRightTicks();

    // Calculer les variations de position en distance et en angle
    float deltaLinPos = (tickRight * m_coeffLongR + ticksLeft * m_coeffLongR)/2;
    float deltaAngPos = (tickRight * m_coeffLongR - ticksLeft * m_coeffLongR);

    // Moyenne des angles pour connaître le cap exact
    const float avgTheta = m_pos.theta + deltaAngPos / 2;

    //Mise à jour de la position du robot en xy et en orientation
    // Convertir rad -> degré ? A revoir
    this->m_pos.x       += deltaLinPos*cos(avgTheta);
    this->m_pos.y       += deltaLinPos*sin(avgTheta);
    this->m_pos.theta   += deltaAngPos;

    // Calcul de la vitesse angulaire et linéaire

    float timestep      = codeurs.getTime() * 0.000001; // en s
    float linVel        = 0; // mm / s
    float angVel        = 0; // mm / s

    if(timestep > 0) {
        linVel = deltaLinPos / timestep;
        angVel = deltaAngPos / timestep;
    }

    // Actualisation de la vitesse linéaire et angulaire du robot
    this->m_linVel = linVel;
    this->m_angVel = angVel;
}
