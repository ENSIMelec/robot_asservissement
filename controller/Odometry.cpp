#include "Odometry.h"
#include "MathUtils.h"

using namespace std;

/**
 * Odométrie constructeur
 * Méthode de régalage des coeffs:
 *  - TICK_TO_MM = faire avancer le robot 1m
 *  - TICK_TO_RAD = faire tourner le robot 360 sur lui même plusieurs fois
 *
 * @param codeurs
 */
Odometry::Odometry(ICodeurManager &codeurs) : m_codeurs(codeurs) {

    //   double CoeffGLong = 0.196370477796;
    //   double CoeffDLong = 0.196232031648;
    //   double CoeffGAngl = 0.035746063822912;
    ///  double CoeffDAngl = 0.035766143278026;

    // Coefficient roue distance
    //this->TICK_RIGHT_TO_MM = 0.223313;
    //this->TICK_LEFT_TO_MM = 0.223613;

    this->TICK_RIGHT_TO_MM  = 0.192901235;
    this->TICK_LEFT_TO_MM   = 0.193259122;

    // Coefficient roue angle (à revérifier)
    //this->TICK_RIGHT_TO_RAD = 0.00133627931;
    //this->TICK_LEFT_TO_RAD  = 0.00135471869;

    this->TICK_RIGHT_TO_RAD = 0.00134527931;
    this->TICK_LEFT_TO_RAD  = 0.00134551869;
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
    float ticksLeft = m_codeurs.getLeftTicks();
    float ticksRight = m_codeurs.getRightTicks();

    // Conversion de distance pour chaque roue parcouru de tick en mm
    float distanceRight = ticksRight * TICK_RIGHT_TO_MM;
    float distanceLeft = ticksLeft * TICK_LEFT_TO_MM;

    m_totalTicksL += ticksLeft;
    m_totalTicksR += ticksRight;

    // Calculer les variations de position en distance et en angle

    // distance parcourue depuis la position de départ jusqu’à l’instant présent.
    float dDistance = (distanceRight + distanceLeft)/2;
    m_dDistance = dDistance;
    m_distance += dDistance;

    //  dAngle = (position_roue_D – position_roue_G) / entraxe
    // Calcul de la différence du nombre de tic entre chaque roue (appx. gauss)
    float dAngle = (ticksRight * TICK_RIGHT_TO_RAD - ticksLeft * TICK_LEFT_TO_RAD) / 2;
    m_dAngle = dAngle;

    // <!> m_pos.theta l'angle initiale
    // Moyenne des angles pour connaître le cap exact
    float avgTheta = m_pos.theta + dAngle/2;
    m_dOrientation = avgTheta;

    //Mise à jour de la position du robot en xy et en orientation
    this->m_pos.x       += dDistance*cosf(avgTheta); // dAngle?
    this->m_pos.y       += dDistance*sinf(avgTheta);
    this->m_pos.theta   += dAngle;

    if(this->m_pos.theta >= M_PI*2 || this->m_pos.theta <= -M_PI*2)
        this->m_pos.theta = 0;

    // Calcul de la vitesse angulaire et linéaire
    // Actualisation du temps
    this->m_lastTime = MathUtils::micros2sec(m_codeurs.getTime());

    float timestep      = m_lastTime; // micros -> s
    float linVel        = 0; // mm / s
    float angVel        = 0; // rad / s

    if(timestep > 0) {
        linVel = dDistance / timestep;
        angVel = dAngle / timestep;
    }

    // Actualisation de la vitesse linéaire et angulaire du robot
    this->m_linVel = linVel;
    this->m_angVel = angVel;
}

float Odometry::getDeltaAngle() const {
    return m_dAngle;
}

float Odometry::getTotalTicksL() const {
    return m_totalTicksL;
}

float Odometry::getTotalTicksR() const {
    return m_totalTicksR;
}

float Odometry::getDeltaDistance() const {
    return m_dDistance;
}
float Odometry::getDeltaOrientation() const {
    return m_dOrientation;
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