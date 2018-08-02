#pragma once

#include "chai3d.h"
#include "trakSTAR.h"
#include "gripperChaiDevice.h"

#define MAGTRACKER
#define NUM_TRACKERS 1

using namespace std;

class magTrackerThread
{
public:

	explicit magTrackerThread(int a_trackerNum);
	~magTrackerThread();
	
	int trackerNum;
	bool trackingOn;


	// thread safety
	chai3d::cMutex m_magTrackerLock;

	void initialize();
	void CheckTrackerPoses();
	
	// device, pair with haptics thread
	chai3d::cGenericHapticDevicePtr* m_chaiMagDevice;
	void pairWithHapticsThread(chai3d::cGenericHapticDevicePtr *a_chaiMagDevice);

	chai3d::cPrecisionClock runTimer;

	// magnetic tracker variables
	CSystem     ATC3DG; // a pointer to a single instance of the system class
	CSensor     *pSensor; // a pointer to an array of sensor objects
	CXmtr       *pXmtr; // a pointer to an array of transmitter objects
	CBoard      *pBoard; // a pointer to an array of board objects
	int         errorCode;
	int         sensorID;
	int         transmitterID;
	short       id;
	int         numberBytes;
	int         i;
	double      measFreq;
	DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD record;


	void run();
};

