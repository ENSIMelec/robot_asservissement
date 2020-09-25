#pragma once

class ServoManager
{

public:
	 
	ServoManager(int i2c_servo);
	void servoAction(int servo, int action );
	
protected :

	int i2c_servo;


};


