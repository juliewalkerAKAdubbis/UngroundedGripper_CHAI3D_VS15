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
	m_thZero = { PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, 0.0 };
	m_clk = new cPrecisionClock();
	m_th = { PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, 0.0 };
	m_thDes = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thErr = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdot = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdotDes = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdotErr = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thErrInt = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_T = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	gripperLength = 0.02;		// [m]
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
	cout << "Move pantographs so that the upper links are straight. Hold gripper at 0 degree angle." << endl;
	Sleep(4000);
	cout << "Starting Calibration now ... ";

	// save encoder counts at calibration configuration
	//for (int i = 0; i < NUM_ENC; i++) {
	//	m_thZero[i] = getCounts((uint)i);
	//}
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
	m_thumbForce = a_thumbForce;		// will only use x and z components (tangent to fingerpad)
	m_fingerForce = a_fingerForce;		// will only use x and z components (tangent to fingerpad)


	// filter gripper force
	if (abs(m_thumbForce.y()) > 0.0 && abs(m_fingerForce.y()) > 0.0) {
		m_gripForce = abs(a_thumbForce.y()) + abs(a_fingerForce.y());
		m_gripForce = F_FILT*(m_gripForce)+(1 - F_FILT)*m_gripForce_last;	
	}
	else {
		m_gripForce = 0.0;
	}
	m_gripForce_last = m_gripForce;

	m_gripperLock.release();
	
		m_T[4] = m_gripForce*gripperLength;

		// calculate desired position for each pantograph in pantograph class and solve for thDes with inverse kinematics
		pThumb.setPos(m_thumbForce);
		pIndex.setPos(m_fingerForce);
	

	//// set motor commands 
	m_thDes[0] = pIndex.m_thDes.x();
	m_thDes[1] = pIndex.m_thDes.y();
	m_thDes[2] = pThumb.m_thDes.x();		// ------------ TO DO: CONFIRM ORDER (ONE PANTOGRAPH IS MIRRORED) ------------
	m_thDes[3] = pThumb.m_thDes.y();

	// DEBUG 
	if (0) {
		m_gripperLock.acquire();
		//cout << "Index Forces: " << m_fingerForce.x() << "   " << -(m_fingerForce.z()) << "     ";
		//cout << "Index DesiredPosition: " << pIndex.m_posDes.x() << ", " << pIndex.m_posDes.z() << "     ";
		cout << "Index DesiredAngles: " << cRadToDeg(m_thDes[0]) << ", " << cRadToDeg(m_thDes[1]) << "     ";
		cout << "Thumb DesiredAngles: " << cRadToDeg(m_thDes[2]) << ", " << cRadToDeg(m_thDes[3]) << "     ";
		//cout << "   Gripper Force: " << m_gripForce << endl;
		cout << endl;
		m_gripperLock.release();
	}
}


void gripper::getState()
{
	// declare "memory" variables for calculations
	double tLast = m_t;
	vector<double> thLast = m_th;
	vector<double> thdotLast = m_thdot;
	vector<double> thErrIntLast = m_thErrInt;
	m_gripperLock.acquire();

	// get current joint positions/errors
	for (int i = 0; i < NUM_MTR; i++) {
		m_th[i] = getAngle(i) + m_thZero[i];
	}

	for (int i = 0; i < NUM_MTR; i++) {
		m_thErr[i] = m_thDes[i] - m_th[i]; //angleDiff(m_thDes[i], m_th[i]);
	}
	
	// calculate velocities/errors
	m_t = m_clk->getCurrentTimeSeconds();
	for (int i = 0; i < NUM_MTR; i++) {
		m_thdot[i] = V_FILT*(angleDiff(m_th[i], thLast[i]) / (m_t - tLast)) + (1 - V_FILT)*thdotLast[i];
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
	m_gripperLock.release();
}


void gripper::motorLoop(void)
{
	//m_gripperLock.acquire();
	getState();
	// calculate error
	for (int i = 0; i < NUM_MTR-1; i++) {  // one joint at a time, only for pantograph motors
		m_T[i] = m_Kp[i]*m_thErr[i] + m_Kd[i]*m_thdotErr[i] +m_Ki[i]*m_thErrInt[i];
	}
	//DEBUG
	//cout << cRadToDeg(m_th[0]) << "   " << cRadToDeg(m_th[1]) << "           " << m_T[0] << "   " << m_T[1] <<  "  " << endl;
	//cout << m_thErr[0]*180/PI << "   " << m_th[0] * 180 / PI << "   " << m_T[0] << "    " ;
	//cout << m_th[0] << "   " << m_th[1] << endl;

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
	cout << "Setting all Voltages to 0 V" << endl;
	Sleep(1000);
	disconnectFromS826();

	m_clk->stop();
	m_gripperReady = false;
	return(C_SUCCESS);
}

bool gripper::disableCtrl(void) {
	for (int i = 0; i < NUM_MTR; i++) {
		setVolts(i, 0.0);
	}
	Sleep(1000);
	return(C_SUCCESS);
}
