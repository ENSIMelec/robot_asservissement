#include "Odometry.h"

using namespace std;

/**
 * Odométrie constructeur
 * Méthode de régalage des coeffs:
 *  - TICK_TO_MM = faire avancer le robot 1m
 *  - TICK_TO_RAD = faire tourner le robot 360 sur lui même plusieurs fois
 *
 * @param codeurs
 */

Odometry::Odometry(ICodeurManager &codeurs, Config& config) : m_codeurs(codeurs), m_config(config) {


    // Coefficient roue distance
//    this->TICK_RIGHT_TO_MM  = 0.192901235;
//    this->TICK_LEFT_TO_MM   = 0.193259122;
//    // Coefficient roue angle (à revérifier)
//    this->TICK_RIGHT_TO_RAD = 0.00134527931;
//    this->TICK_LEFT_TO_RAD  = 0.00134551869;

    // nouvelle approche odométrie
    this->PERIM_ROUE = 36*M_PI; //Diametre * PI
    this->RESOLUTION = 516;
    this->COEF_CORRECTEUR = m_config.getCoeffCorrecteur();
    this->ENTRAXE = 288;

}

/**
* @brief Calcule la nouvelle position et la nouvelle vitesse.
* détermine la nouvelle vitesse instantanée et la nouvelle position par approximation de segment de droite
 * @TODO: average left, and right speed .
 */
void Odometry::update() {


    // récupérer les tics des codeurs + réinitialisation
    m_codeurs.readAndReset();

    // Récupéreration des tics codeurs
    long int ticksLeft = m_codeurs.getLeftTicks();
    long int ticksRight = m_codeurs.getRightTicks();

    // calculer la distance de effectué par chaque roue
    float distanceLeft = ticksLeft * (PERIM_ROUE/RESOLUTION);
    float distanceRight = ticksRight * ((COEF_CORRECTEUR*PERIM_ROUE)/RESOLUTION);


    // Calculer les variations de position en distance et en angle

    // distance parcourue depuis la position de départ jusqu’à l’instant présent.
    m_dDistance = (distanceRight + distanceLeft) / 2;
    // Calcul de la différence du nombre de tic entre chaque roue (appx. gauss)
    float dAngle = (distanceRight - distanceLeft) / ENTRAXE;

    // <!> m_pos.theta l'angle initiale
    // Moyenne des angles pour connaître le cap exact
    m_dAvgTheta = m_pos.theta + dAngle/2;

    //Mise à jour de la position du robot en xy et en angle

    this->m_pos.x       += m_dDistance * cosf(m_dAvgTheta); // dAngle?
    this->m_pos.y       += m_dDistance * sinf(m_dAvgTheta);
    this->m_pos.theta   += MathUtils::inrange(dAngle,-M_PI, M_PI);

    // shit! => borner l'angle entre [-pi, pi]
    //    if(this->m_pos.theta >= M_PI*2 || this->m_pos.theta <= -M_PI*2)
    //        this->m_pos.theta = 0;

    // Calcul de la vitesse angulaire et linéaire
    // Actualisation du temps
    this->m_lastTime = m_codeurs.getTime();

    float timestep      = MathUtils::micros2sec(m_lastTime); // micros -> s
    float linVel        = 0; // mm / s
    float angVel        = 0; // rad / s

    if(timestep > 0) {
        linVel = m_dDistance / timestep;
        angVel = dAngle / timestep;
    }

    // Actualisation de la vitesse linéaire et angulaire du robot
    this->m_linVel = linVel;
    this->m_angVel = angVel;

    // Actualisation du total distance parcouru
    distance_total_update(ticksLeft, ticksRight);
}

/**
 * @brief Debug purpose
 */
void Odometry::debug() {
    cout << "===========DEBUG ODOMETRY============" << endl;
    cout << "[DATA CODEUR][TICS] : Gauche:" << m_codeurs.getLeftTicks() << " Droit: " << m_codeurs.getRightTicks() << endl;
    cout << "[DATA CODEUR][TOTAL TICS] : Gauche:" << getTotalTicksL() << " Droit: " << getTotalTicksR() << endl;
    cout << "[DATA CODEUR][LAST TIME] : " << getLastTime() << " (s)" << endl;
    cout << "[ODOMETRY][POSITION] : X:" << getPosition().x << " Y: " << getPosition().y << " Theta: " <<  MathUtils::rad2deg(getPosition().theta) << " °" << endl;
    cout << "[ODOMETRY][DISTANCE PARCOURU EN LASTTIME (mm)] : " << getDeltaDistance() << endl;
    cout << "[ODOMETRY][ROTATION EFFECTUE EN LASTTIME (rad)] : " << getDeltaOrientation() << endl;
    cout << "[ODOMETRY][VITESSE]: Vitesse angulaire (rad/s) : " << getAngVel() << " Vitesse Linéaire (mm/s) : " << getLinVel() << endl;
    cout << "[ODOMETRY][TOTAL DISTANCE] (cm): " << getTotalDistance() / 10 << endl;
    cout << "=======================" << endl;

}

void Odometry::distance_total_update(int long ticksLeft, int long ticksRight) {

    m_totalTicksL += ticksLeft;
    m_totalTicksR += ticksRight;

    m_totalDistance += m_dDistance;
}


