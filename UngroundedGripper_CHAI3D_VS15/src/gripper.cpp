#include "gripper.h"
#include "Windows.h"


gripper::gripper(void) {
	// exoskeleton available for connection
	m_gripperAvailable = true;
	m_gripperReady = false;
	m_error = false;
	m_errMessage = "";
	
	for (int i = 0; i < NUM_ENC; i++) { m_thZero[i] = 0; }

	//attach two pantographs;
	pantograph pThumb(fingers::thumb);
	pantograph pIndex(fingers::index);

	//set up the grip force motor

}

gripper::~gripper(void){
	// disconnect from S826 and free dynamically allocated memory
	if (disconnect()) {
		delete m_clk;
	}
}


bool gripper::connect()
	{
		// check if gripper is available/has been opened already
		if (!m_gripperAvailable) return(C_ERROR);
		if (m_gripperReady)      return(C_ERROR);

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
		m_gripperReady = true;
		return(C_SUCCESS);
	}


bool gripper::calibrate(void) {
	cout << "Ready to Calibrate?" << endl << endl;
	cout << "Move pantographs so that the upper links are straight. Hold gripper at 45 degree angle." << endl;

	// save encoder counts at calibration configuration
	for (int i = 0; i < NUM_ENC; i++) {
		m_thZero[i] = getCounts((uint)i);
	}
	return(C_SUCCESS);

}


bool gripper::disconnect(void){
	// check that exoskeleton is open
	if (!m_gripperReady) return(C_ERROR);

	// set motor torques to zero and disconnect from S826
	disableCtrl();
	disconnectFromS826();

	m_clk->stop();
	m_gripperReady = false;
	return(C_SUCCESS);
}

bool gripper::disableCtrl(void) {
	// set all motors to zero  ****************** TO DO ***************************
	return(C_SUCCESS);
}


bool gripper::sendCommand(pantograph& pThumb, pantograph& pIndex)
{
	// set force to grip motor

	// send command to pantrograph components to execute forces
	pThumb.sendCommand();
	pIndex.sendCommand();
	return(C_SUCCESS);
}

// receive force and torque information in the gripper's local coordinate frame
void gripper::setForcesAndTorques(cVector3d a_force, cVector3d a_torque, double a_gripForce, cVector3d a_thumbForce, cVector3d a_fingerForce) {
	m_gripperLock.acquire();
	m_force = a_force;
	m_torque = a_torque;
	m_gripForce = a_gripForce;

	// in local coordinates
	m_thumbForce = a_thumbForce;
	m_fingerForce = a_fingerForce;
	m_gripForce = a_thumbForce.y() + a_fingerForce.y();
	
	
	cout << m_gripForce << endl;
	m_gripperLock.release();

	// calculate desired gripper motor action																	------TO DO-----
	// calculate desired position for each pantograph in pantograph class
	// set motor commands 
}
