#pragma once
#include "chai3d.h"
#include "motorcontrol.h"
#include <cmath>
#include <array>

#define A_FILT         0.5      // weight for velocity filtering
#define INT_CLMP       750      // maximum allowed integrated error
#define THRESH         0.01     // threshold for angle equality
#define PI  3.14159

enum fingers {
	thumb,
	index
};


class pantograph {

public:

	bool m_error;                           // TRUE = problem with exo while running
	std::string m_errMessage;               // error message
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock* m_clk;         // pointer to clock for computing velocity
											// exoskeleton available for connection
	//bool m_pantographAvailable;
	//bool m_pantographReady;



	double lengths[5];	// upper left, bottom left, bottom right, upper right, top bar
	double m_finger;		// index or thumb (mirrored x position values)

	chai3d::cVector3d m_th;                 // current joint angles [rad]
	chai3d::cVector3d m_thDes;              // desired joint angles [rad]
	chai3d::cVector3d m_thErr;              // joint-angle error [rad]
	chai3d::cVector3d m_thdot;              // current joint velocities [rad/s]
	chai3d::cVector3d m_thdotDes;           // desired joint velocities [rad/s]
	chai3d::cVector3d m_thdotErr;           // joint-velocity error [rad/s]
	chai3d::cVector3d m_thErrInt;           // integrated joint angle error [rad*s]
	chai3d::cVector3d m_pos;                // current end-effector position [m]
	chai3d::cVector3d m_posDes;             // desired end-effector position [m]
	chai3d::cVector3d m_posErr;             // end-effector position error [m]
	chai3d::cVector3d m_vel;                // current end-effector linear velocity [m/s]
	chai3d::cVector3d m_velDes;             // desired end-effector linear velocity [m/s]
	chai3d::cVector3d m_velErr;             // end-effector velocity error [m/s]
	chai3d::cVector3d m_posErrInt;          // integrated end-effector position error [m*s]
	

	chai3d::cMutex m_pantographLock;
	chai3d::cVector3d m_Kp;
	chai3d::cVector3d m_Kd;
	chai3d::cVector3d m_Ki;

	pantograph(fingers a_finger);
	~pantograph();

//	void getState();
	bool sendCommand();

//	chai3d::cVector3d forwardKin(chai3d::cVector3d a_th);
//	chai3d::cVector3d inverseKin(chai3d::cVector3d a_pos);

protected:
	bool m_pantographAvailable;            // TRUE = exoskeleton instance has been created
	bool m_pantographReady;                // TRUE = connection to exoskeleton successful

//	chai3d::cVector3d getAngles();
//	chai3d::cMatrix3d Jacobian(chai3d::cVector3d a_th);
	double angleDiff(double a_thA, double a_thB);
	chai3d::cVector3d vecDiff(chai3d::cVector3d a_vecA, chai3d::cVector3d a_vecB);
};
