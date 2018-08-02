#pragma once
#include "chai3d.h"
#include "motorcontrol.h"
#include "Windows.h"
#include <cmath>
#include <array>

using namespace chai3d;
using namespace std;

#define NUM_ENC 5  // number of encoders (0 = index distal, 1 = index proximal, 2 = thumb proximal, 3 = thumb distal, 5 = grip force)
#define NUM_MTR 5  // number of motors (0 = index distal, 1 = index proximal, 2 = thumb proximal, 3 = thumb distal, 5 = grip force)

class Gripper {

public:
	//Subject* subject;

	bool time; // time
	chai3d::cPrecisionClock* clk;


	chai3d::cVector3d m_pos;                // current end-effector position [m]
	chai3d::cVector3d m_posDes;             // desired end-effector position [m]
	chai3d::cVector3d m_posErr;             // end-effector position error [m]
	chai3d::cVector3d m_vel;                // current end-effector linear velocity [m/s]
	chai3d::cVector3d m_velDes;             // desired end-effector linear velocity [m/s]
	chai3d::cVector3d m_velErr;             // end-effector velocity error [m/s]
	chai3d::cVector3d m_posErrInt;          // integrated end-effector position error [m*s]

	chai3d::cVector3d Ki;              // integral gains for joint-space control
	chai3d::cVector3d Kp;              // proportional gains for task-space control
	chai3d::cVector3d Kd;              // derivative gains for task-space control

	chai3d::cVector3d T;                  // desired torques [N*m]
	chai3d::cVector3d F;                  // desired end-effector force [N]


	bool connect();
	bool disconnect();
	void calibrate();
	void getState();
	bool sendCommand();


	//chai3d::cVector3d forwardKin(chai3d::cVector3d a_th);
	//chai3d::cVector3d inverseKin(chai3d::cVector3d a_pos);

protected:
	bool gripperAvailable;            // TRUE = exoskeleton instance has been created
	bool gripperReady;                // TRUE = connection to exoskeleton successful
	//void disableCtrl() { setJntTorqs(chai3d::cVector3d(0.0, 0.0, 0.0)); }


};