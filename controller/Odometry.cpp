#include "Odometry.h"

using namespace std;

Odometry::Odometry(ICodeurManager &codeurs) : m_codeurs(codeurs) {

    // Coefficient roue linéaire
    this->m_coeffLongL = 0.223313;
    this->m_coeffLongR = 0.223613;

    // Coefficient roue angle
    this->m_coeffAngL = 0.5460;
    this->m_coeffLongR = 0.4343;
}

/**
 * Odométrie
 * Calcul de la position et orientation du robot et vitesse entre deux intervalle de temps
 */

void Odometry::process() {

    // récupérer les tics des codeurs + réinitialisation
    m_codeurs.readAndReset();

    // récupérer les tics codeurs
    float ticksLeft = m_codeurs.getLeftTicks();
    float ticksRight = m_codeurs.getRightTicks();

    m_totalTicsL += ticksLeft;
    m_totalTicsR += ticksRight;

    // Calculer les variations de position en distance et en angle
    float deltaLinPos = (ticksRight * m_coeffLongR + ticksLeft * m_coeffLongR)/2;
    float deltaAngPos = (ticksRight * m_coeffLongR - ticksLeft * m_coeffLongR);

    // Moyenne des angles pour connaître le cap exact
    const float avgTheta = m_pos.theta + deltaAngPos / 2;

    //Mise à jour de la position du robot en xy et en orientation
    // Convertir rad -> degré ? A revoir
    this->m_pos.x       += deltaLinPos*cos(avgTheta * 180/M_PI);
    this->m_pos.y       += deltaLinPos*sin(avgTheta * 180/M_PI);
    this->m_pos.theta   += deltaAngPos;

    // Calcul de la vitesse angulaire et linéaire

    float timestep      = m_codeurs.getTime() * 0.000001; // en s
    float linVel        = 0; // mm / s
    float angVel        = 0; // rad / s

    if(timestep > 0) {
        linVel = deltaLinPos / timestep;
        angVel = deltaAngPos / timestep;
    }
    // Actualisation du temps
    this->m_lastTime = m_codeurs.getTime();

    // Actualisation de la vitesse linéaire et angulaire du robot
    this->m_linVel = linVel;
    this->m_angVel = angVel;
}

float Odometry::getTotalTicsL() const {
    return m_totalTicsL;
}

float Odometry::getTotalTicsR() const {
    return m_totalTicsR;
}

void Odometry::printData() {

    cout << "[DATA CODEUR][TOTAL TICS] : Gauche:" << getTotalTicsL() << " Droit: " << getTotalTicsR() << endl;
    cout << "[DATA CODEUR][POSITION] : X:" << getPosition().x << " Y: " << getPosition().y << " Theta: " <<  getPosition().theta << endl;
    cout << "[DATA CODEUR][LASTTIME] : " << getLastTime() << endl;
    cout << "[DATE CODEUR][VITESSE]; Vitesse angulaire(rad/s) : " << getAngVel() << " Vitesse Linéaire (mm/s) : " << getLinVel() << endl;
    cout << "=======================" << endl;

}
