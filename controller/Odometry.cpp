#include "Odometry.h"

using namespace std;

Odometry::Odometry(ICodeurManager &codeurs) : m_codeurs(codeurs) {

    // Coefficient roue linéaire
    this->TICK_RIGHT_TO_MM = 0.223313;
    this->TICK_LEFT_TO_MM = 0.223613;

    // Coefficient roue angle
    this->TICK_RIGHT_TO_RAD = 0.5460;
    this->TICK_LEFT_TO_RAD = 0.4343;
}

/**
 * Odométrie
 * Calcul de la position et orientation du robot et vitesse entre deux intervalle de temps
 */
void Odometry::process() {

    // récupérer les tics des codeurs + réinitialisation
    m_codeurs.readAndReset();

    // Récupéreration des tics codeurs
    float ticksLeft = m_codeurs.getLeftTicks();
    float ticksRight = m_codeurs.getRightTicks();

    // Conversion de distance pour chaque roue parcouru de tick en mm
    float distanceRight = ticksRight * TICK_RIGHT_TO_MM;
    float distanceLeft = ticksLeft * TICK_LEFT_TO_MM;

    m_totalTicksL += ticksLeft;
    m_totalTicksR += ticksRight;

    // Calculer les variations de position en distance et en angle  Chemin parcouru pendant le dernier intervalle de temps)
    float dDistance = (distanceRight + distanceLeft)/2;

    //  dAngle = Өo + (position_roue_D – position_roue_G)
    //  avec Өo représentant l’orientation initiale du robot
    float dAngle = (ticksRight * TICK_RIGHT_TO_RAD - ticksLeft * TICK_LEFT_TO_RAD);

    // <!> m_pos.theta l'angle initiale
    // Moyenne des angles pour connaître le cap exact
    float avgTheta = m_pos.theta + dAngle / 2;

    //Mise à jour de la position du robot en xy et en orientation
    // Convertir rad -> degré ? A revoir
    this->m_pos.x       += dDistance*cos(avgTheta);
    this->m_pos.y       += dDistance*sin(avgTheta);
    this->m_pos.theta   += dAngle;

    // Calcul de la vitesse angulaire et linéaire

    float timestep      = m_codeurs.getTime() * 0.000001; // en s
    float linVel        = 0; // mm / s
    float angVel        = 0; // rad / s

    if(timestep > 0) {
        linVel = dDistance / timestep;
        angVel = dAngle / timestep;
    }
    // Actualisation du temps
    this->m_lastTime = m_codeurs.getTime();

    // Actualisation de la vitesse linéaire et angulaire du robot
    this->m_linVel = linVel;
    this->m_angVel = angVel;
}

float Odometry::getTotalTicksL() const {
    return m_totalTicksL;
}

float Odometry::getTotalTicksR() const {
    return m_totalTicksR;
}
/**
 * Debug purpose
 */
void Odometry::printData() {

    cout << "[DATA CODEUR][TOTAL TICS] : Gauche:" << getTotalTicksL() << " Droit: " << getTotalTicksR() << endl;
    cout << "[DATA CODEUR][POSITION] : X:" << getPosition().x << " Y: " << getPosition().y << " Theta: " <<  getPosition().theta << endl;
    cout << "[DATA CODEUR][LAST TIME] : " << getLastTime() << endl;
    cout << "[DATE CODEUR][VITESSE]; Vitesse angulaire(rad/s) : " << getAngVel() << " Vitesse Linéaire (mm/s) : " << getLinVel() << endl;
    cout << "=======================" << endl;

}
