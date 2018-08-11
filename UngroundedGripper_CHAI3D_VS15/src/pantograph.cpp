#include "pantograph.h"

using namespace std;
using namespace chai3d;


pantograph::pantograph() {

	// exoskeleton available for connection
	m_pantographAvailable = true;
	m_pantographReady = false;
	m_error = false;
	m_errMessage = "";

	//m_finger = a_finger;
	

	// initialize kinematic variables
	m_t = 0;
	m_th = cVector3d(0.0, 0.0, 0.0);
	m_thDes = cVector3d(0.0, 0.0, 0.0);


}

pantograph::~pantograph(void) {
	//set all motors to zero 
}


void pantograph::setPos(cVector3d a_force){		//double a_x, double a_z) {
	m_posDes.x(a_force.x() / k_skin);
	m_posDes.z(a_force.z() / k_skin);
	inverseKinematics();
}

void pantograph::inverseKinematics() {
	//calculate desired angles from matlab code
	double xx = m_posDes.x();
	double yy = m_posDes.z();

	// Distances to point from each motor hub
	//double P1P3 = sqrt( (double)(m_pos.x)*(double)(m_pos.x) + (double)(m_pos.z)*(double)(m_pos.z));
	double P1P3 = sqrt(xx*xx + yy*yy);
	double P5P3 = sqrt((xx + len[4])*(xx + len[4]) + yy*yy);
	if ((P1P3 > (len[0] + len[1])) || (P5P3 > (len[3] + len[2]))) {	// longer than length of arms
		cout << "Pantograph " << m_finger << " out of workspace" << endl;
		m_thDes.set(center[0], center[1], 0.0);		// center pantograph point
		return;
	}

	// intermediate terms
	double alpha1 = acos((len[1] * len[1] - len[0] * len[0] - P1P3*P1P3) / (-2 * len[0] * P1P3));
	if (alpha1 < 0) {
		cout << "Pantograph" << m_finger << " bad configuration, alpha1 < 0" << endl;
		m_thDes.set(center[0], center[1], 0.0);		// center pantograph point
		return;
	}

		double beta1 = atan2(yy, -xx);
	double beta5 = acos((len[2]*len[2] - len[3]*len[3] - P5P3*P5P3) / (-2 * len[3] * P5P3));

	if(beta5 < 0){
		cout << "Pantograph" << m_finger << " bad configuration, beta5 < 0" << endl;
		m_thDes.set(center[0], center[1], 0.0);		// center pantograph point
		return;
	}
		double alpha5 = atan2(yy, xx + len[4]);

		// set angles in thDes
		m_thDes.set(PI - alpha1 - beta1, alpha5 + beta5, 0.0);

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

