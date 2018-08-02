#pragma once
#include "chai3d.h"
#include "motorcontrol.h"
#include "pantograph.h"


#include <cmath>
#include <array>

using namespace chai3d;
using namespace std;

#define NUM_ENC 5	// nunber of encoders 2 on each finger and gripper motor = 5
#define NUM_MTR 5 

////------------------------------------------------------------------------------
//// ERROR CONSTANTS
//const bool C_ERROR = false;			//! Function returns with an error.
//const bool C_SUCCESS = true;		//! Function returns successfully.
////------------------------------------------------------------------------------


class gripper : public cGenericHapticDevice
{

public:


	bool m_error;                           // TRUE = problem with exo while running
	std::string m_errMessage;               // error message
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock* m_clk;         // pointer to clock for computing velocity
	bool m_gripperAvailable;
	bool m_gripperReady;
	chai3d::cMutex m_gripperLock;

	int m_thZero[NUM_ENC];                  // zero angles for motor-angle measurement [counts, in motor space]


	gripper();
	~gripper();
	bool connect();
	bool disconnect();
	bool disableCtrl();
	bool calibrate();


	chai3d::cVector3d force;
	chai3d::cVector3d torque;
	double gripforce;

	//void getState();

	bool sendCommand(pantograph& pThumb, pantograph& pIndex);
};