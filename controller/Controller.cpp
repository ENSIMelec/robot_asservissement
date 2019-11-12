//
// Created by Taoufik on 12/11/2019.
//

#include "Controller.h"
#include "PID.h"

Controller::Controller()
{
    // Init PID Controllers

    // Speed Controller
    m_leftSpeedPID = PID(0,0,0,0,500);
    m_rightSpeedPID = PID(0,0,0,0,500);

    // Translation Controller
    m_translationPID = PID(0,0,0,0,500);
    m_rotationPID = PID(0,0,0,0, 2*M_PI);


}
void Controller::process()
{

    //Mise à jour de la position et de la vitesse du robot (Odométrie)

    // Calcul de la distance et de l'angle à fair pour aller au prochain point

    // Déplacement en fonction du type du point
}

void Controller::rotate() {

}
/**
 * Déplacement
 * @param amount
 */
void Controller::translate(double distance)
{
    // reset PID
    m_translationPID.resetErrors();

    // préciser la consigne de distance
    m_targetDistance = distance;

}
