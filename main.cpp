#include <iostream>
#include "controller/FakeCodeur.h"
#include "controller/Odometry.h"
#include "controller/Controller.h"

using namespace std;

int main() {


    FakeCodeur fakeCodeur;

    MoteurManager motor(8);

    Odometry odometry(fakeCodeur);
    Controller controller(fakeCodeur, motor);

    controller.gotoPoint(200,300,0);
    while(1) {

        controller.update();

        sleep(2);
    }
    return 0;
}
