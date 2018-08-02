#include "pantograph.h"

using namespace std;
using namespace chai3d;


pantograph::pantograph(fingers a_finger) {

	// exoskeleton available for connection
	m_pantographAvailable = true;
	m_pantographReady = false;
	m_error = false;
	m_errMessage = "";

	m_finger = a_finger;

	// initialize kinematic variables
	m_t = 0;
	m_th = cVector3d(0.0, 0.0, 0.0);
	m_thDes = cVector3d(0.0, 0.0, 0.0);
	m_thErr = cVector3d(0.0, 0.0, 0.0);
	m_thdot = cVector3d(0.0, 0.0, 0.0);
	m_thdotDes = cVector3d(0.0, 0.0, 0.0);
	m_thdotErr = cVector3d(0.0, 0.0, 0.0);
	m_thErrInt = cVector3d(0.0, 0.0, 0.0);
	m_pos = cVector3d(0.0, 0.0, 0.0);
	m_posDes = cVector3d(0.0, 0.0, 0.0);
	m_posErr = cVector3d(0.0, 0.0, 0.0);
	m_vel = cVector3d(0.0, 0.0, 0.0);
	m_velDes = cVector3d(0.0, 0.0, 0.0);
	m_velErr = cVector3d(0.0, 0.0, 0.0);
	m_posErrInt = cVector3d(0.0, 0.0, 0.0);

	// initialize control variables
	m_Kp = cVector3d{ 1.0, 1.0, 1.0 };
	m_Kd = cVector3d{ 1.0, 1.0, 1.0 };
	m_Ki = cVector3d{ 1.0, 1.0, 1.0 };

}

pantograph::~pantograph(void) {
	//set all motors to zero 
}

bool pantograph::sendCommand(void) {

	// ......................... TO DO ................................ //
	return(C_SUCCESS);
}

double pantograph::angleDiff(double a_thA, double a_thB)
{
	// determine shortest distance between A and B
	double diff1 = fabs(a_thA - a_thB);
	double diff2 = 2 * PI - diff1;
	double diff = fmin(diff1, diff2);

	// determine direction for moving from B to A along shortest path
	if (abs(fmod(a_thB + diff, 2 * PI) - a_thA) < THRESH)  return  1.0*diff;
	else                                                 return -1.0*diff;
}

cVector3d pantograph::vecDiff(cVector3d a_vecA, cVector3d a_vecB)
{
	cVector3d diff;
	for (int i = 0; i < 3; i++) {
		diff(i) = angleDiff(a_vecA(i), a_vecB(i));
	}
	return diff;
}
