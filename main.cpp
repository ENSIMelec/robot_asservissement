#include <iostream>
#include "Asservissement.h"
#include "FakeCodeur.h"
#include "controller/Odometry.h"

using namespace std;

int main() {

    MoteurManager moteurManager(8);
    FakeCodeur fakeCodeur;

    Asservissement asservissement(moteurManager,fakeCodeur);

    Odometry odometry(fakeCodeur);
    // réinitialisation de la position
    odometry.setPosition(0,0,0);

    cout << "Hello, World!" << endl;
    return 0;
}