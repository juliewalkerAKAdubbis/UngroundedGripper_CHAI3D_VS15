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


	// arm lengths: upper left, bottom left, bottom right, upper right, top bar
	double len[5] = { 13, 11, 11, 13, 15 };		// CORRECT THESE TO MATCH DEVCIE
	double center[2] = { PI / 2, PI / 2 };

	double m_finger;		// index or thumb (mirrored x position values)

	chai3d::cVector3d m_th;                 // current joint angles [rad]
	chai3d::cVector3d m_thDes;              // desired joint angles [rad]
	chai3d::cVector3d m_posDes;             // desired end-effector position [m]


	chai3d::cMutex m_pantographLock;


	pantograph();
	~pantograph();

//	void getState();
	void setPos(chai3d::cVector3d a_force); // double a_x, double a_y);

//	chai3d::cVector3d forwardKin(chai3d::cVector3d a_th);
//	chai3d::cVector3d inverseKin(chai3d::cVector3d a_pos);

protected:
	double k_skin = 10;				// scale force to skin displacement
	bool m_pantographAvailable;            // TRUE = exoskeleton instance has been created
	bool m_pantographReady;                // TRUE = connection to exoskeleton successful
	void inverseKinematics();
//	chai3d::cVector3d getAngles();
//	chai3d::cMatrix3d Jacobian(chai3d::cVector3d a_th);
	double angleDiff(double a_thA, double a_thB);
	chai3d::cVector3d vecDiff(chai3d::cVector3d a_vecA, chai3d::cVector3d a_vecB);
};
