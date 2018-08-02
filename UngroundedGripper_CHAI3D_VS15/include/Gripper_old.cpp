#include "Gripper.h"


Gripper::Gripper() {
	gripperAvailable = true;
	gripperReady = false;

	//initialize kinematic variables *********************** TO DO *******************

	// initialize control variables
	Kp = cVector3d(1.0, 1.0, 1.0);
	Kd = cVector3d(1.0, 1.0, 1.0);
	Ki = cVector3d(1.0, 1.0, 1.0);
}

bool Gripper::connect(void) {
	// check if exoskeleton is available/has been opened already
	if (!gripperAvailable) return(C_ERROR);
	if (gripperReady)      return(C_ERROR);

	// connect to S826
	bool success = connectToS826();
	if (!success) return(C_ERROR);

	// initialize encoders
	for (int i = 0; i < NUM_ENC; i++) {
		success = initEncod((uint)i);
		if (!success) return(C_ERROR);
	}

	// initialize motors
	for (int i = 0; i < NUM_MTR; i++) {
		success = initMotor((uint)i);
		if (!success) return(C_ERROR);
	}

	m_clk->start();
	m_exoReady = true;
	return(C_SUCCESS);
}

bool Gripper::disconnect(void) {

}

void Gripper::calibrate(void) {

}

void Gripper::getState(void) {

}

bool Gripper::sendCommand(void) {

}