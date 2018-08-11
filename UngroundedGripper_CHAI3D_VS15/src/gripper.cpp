#include "gripper.h"
#include "Windows.h"


gripper::gripper(void){	//:  pThumb(fingers::thumb), pIndex(fingers::index)


	// exoskeleton available for connection
	m_gripperAvailable = true;
	m_gripperReady = false;
	m_error = false;
	m_errMessage = "";
	
	// kinematic variables
	m_t = 0;
	for (int i = 0; i < NUM_ENC; i++) { m_thZero[i] = 0; }
	m_clk = new cPrecisionClock();
	m_th = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thDes = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thErr = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdot = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdotDes = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdotErr = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thErrInt = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_T = { 0.0, 0.0, 0.0, 0.0, 0.0 };

	//attach two pantographs;
	pThumb.m_finger = fingers::thumb;
	pIndex.m_finger = fingers::index;

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
	// Sleep(4000);
	cout << "Starting Calibration now ... ";

	// save encoder counts at calibration configuration
	for (int i = 0; i < NUM_ENC; i++) {
		m_thZero[i] = getCounts((uint)i);
	}
	return(C_SUCCESS);

}
//
//
//bool gripper::sendCommand(void)// pantograph& pThumb, pantograph& pIndex)
//{
//	// set force to grip motor
//
//	// send command to pantrograph components to execute forces
//	pThumb.sendCommand();
//	pIndex.sendCommand();
//	return(C_SUCCESS);
//}

// receive force and torque information in the gripper's local coordinate frame
void gripper::setForcesAndTorques(cVector3d a_force, cVector3d a_torque, double a_gripForce, cVector3d a_thumbForce, cVector3d a_fingerForce) {
	m_gripperLock.acquire();
	m_force = a_force;
	m_torque = a_torque;
	m_gripForce = a_gripForce;

	// in local coordinates
	m_thumbForce = a_thumbForce;		// will only use x and z components
	m_fingerForce = a_fingerForce;		// will only use x and z components
	m_gripForce = a_thumbForce.y() + a_fingerForce.y();
	
	
	cout << m_gripForce << endl;
	m_gripperLock.release();

	// ---------------------------------------------------------TO DO----------------------------------------------------------------
	// calculate desired gripper motor action					
	setGripMotorVoltage();

	

	// calculate desired position for each pantograph in pantograph class
	pThumb.setPos(m_thumbForce);
	pIndex.setPos(m_fingerForce);
	// set motor commands 
	m_thDes[0] = pThumb.m_thDes(0);
	m_thDes[1] = pThumb.m_thDes(1);
	m_thDes[2] = pIndex.m_thDes(0);
	m_thDes[3] = pIndex.m_thDes(1);

}

void gripper::setGripMotorVoltage(void){	// calculate grip voltage based on m_gripforce
	// voltage to motor = kP_grip*m_gripforce;

}


void gripper::getState()
{
	// declare "memory" variables for calculations
	static double tLast = m_t;
	vector<double> thLast = m_th;
	static vector<double> thdotLast = m_thdot;
	static vector<double> thErrIntLast = m_thErrInt;

	// get current joint positions/errors
	for (int i = 0; i < NUM_MTR; i++) {
		m_th[i] = getAngle(i, zero[i]);
	}
	for (int i = 0; i < NUM_MTR; i++) {
		m_thErr[i] = angleDiff(m_thDes[i], m_th[i]);
	}

	// calculate velocities/errors
	m_t = m_clk->getCurrentTimeSeconds();
	for (int i = 0; i < NUM_MTR; i++) {
		m_thdot[i] = A_FILT*(angleDiff(m_th[i], thLast[i]) / (m_t - tLast)) + (1 - A_FILT)*thdotLast[i];
		m_thdotErr[i] = m_thdotDes[i] - m_thdot[i];

		// integrate position error
		m_thErrInt[i] = thErrIntLast[i] + m_thErr[i]*(m_t - tLast);
	}

	// clamp integrated error
	for (int i = 0; i < NUM_ENC; i++) {
		if (fabs(m_thErrInt[i]) > INT_CLMP) {
			m_thErrInt[i] = (m_thErrInt[i] / fabs(m_thErrInt[i]))*INT_CLMP;
		}
	}

	// save last values
	tLast = m_t;
	thLast = m_th;
	thdotLast = m_thdot;
	thErrIntLast = m_thErrInt;
}

void gripper::motorLoop(void)
{
	//m_gripperLock.acquire();
	getState();
	// calculate error
	for (int i = 0; i < NUM_MTR-1; i++) {  // one joint at a time

		m_thErr[i] = getAngle(i, zero[i]);
		m_thdotErr[i] = m_thdotDes[i] - m_thdot[i];
		m_T[i] = m_Kp[i]*m_thErr[i] + m_Kd[i]*m_thdotErr[i] +m_Ki[i]*m_thErrInt[i];
	}
	m_T[4] = m_Kp[4] * m_gripForce;	// set gripper motor torque based on gripForce not desired angle

	for (int i = 0; i < NUM_MTR; i++) {
		setTorque(i, m_T[i]);
	}
	//m_gripperLock.release();
}


bool gripper::disconnect(void) {
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
