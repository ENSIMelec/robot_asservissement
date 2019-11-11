//
// Created by Taoufik on 11/11/2019.
//

#ifndef ROBOT_FAKECODEUR_H
#define ROBOT_FAKECODEUR_H


#include "ICodeurManager.h"

class FakeCodeur : public ICodeurManager {

public:
    FakeCodeur();

    void readAndReset() override;
    void reset() override;

protected:
    int leftTicks = 0, rightTicks = 0, tempsLast=0;
    int oldLeftTicks = 0, oldRightTicks = 0, oldTempsLast=0;
};


#endif //ROBOT_FAKECODEUR_H
