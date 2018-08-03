#include "magtracker.h"

magTrackerThread::magTrackerThread(int a_trackerNum)
{
	trackerNum = a_trackerNum;
}

magTrackerThread::~magTrackerThread()
{
	trackingOn = false;
}

void magTrackerThread::run()
{
	while(trackingOn)
	{
		if (runTimer.timeoutOccurred())
		{
			m_magTrackerLock.acquire();
			CheckTrackerPoses();
			m_magTrackerLock.release();
			runTimer.reset();
			runTimer.start();
		}
	}
}

void magTrackerThread::initialize()
{
	trackingOn = true;
	runTimer.setTimeoutPeriodSeconds(0.002);

#ifdef MAGTRACKER
	// initialize the magnetic tracker
	cout << "Initializing the ATC3DG system...\n" << endl;
	errorCode = InitializeBIRDSystem();
	if (errorCode == BIRD_ERROR_SUCCESS) {
		cout << "Initialized ATC3DG system\n" << endl;
	}

	// get configurations
	errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
	pSensor = new CSensor[ATC3DG.m_config.numberSensors];
	for (i = 0; i<ATC3DG.m_config.numberSensors; i++)
	{
		errorCode = GetSensorConfiguration(i, &pSensor[i].m_config);
		if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
		cout << "Got sensors configuration\n" << endl;
	}
	pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
	for (i = 0; i<ATC3DG.m_config.numberTransmitters; i++)
	{
		errorCode = GetTransmitterConfiguration(i, &pXmtr[i].m_config);
		if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
		cout << "Got transmitters configuration\n" << endl;
	}

	measFreq = 80.0;
	// set parameters for recording
	SET_SYSTEM_PARAMETER(SELECT_TRANSMITTER, 0);
	SET_SYSTEM_PARAMETER(POWER_LINE_FREQUENCY, 60.0);
	SET_SYSTEM_PARAMETER(AGC_MODE, SENSOR_AGC_ONLY);
	SET_SYSTEM_PARAMETER(MEASUREMENT_RATE, measFreq);
	SET_SYSTEM_PARAMETER(MAXIMUM_RANGE, 72.0);
	SET_SYSTEM_PARAMETER(METRIC, true);

	for (sensorID = 0; sensorID<ATC3DG.m_config.numberSensors; sensorID++) {
		SET_SENSOR_PARAMETER(sensorID, DATA_FORMAT, DOUBLE_POSITION_MATRIX_TIME_STAMP);
		{
			// initialize a structure of angles
			DOUBLE_ANGLES_RECORD anglesRecord = { 0, 0, 0 };
			SET_SENSOR_PARAMETER(sensorID, ANGLE_ALIGN, anglesRecord);
		}
		SET_SENSOR_PARAMETER(sensorID, HEMISPHERE, FRONT);
		SET_SENSOR_PARAMETER(sensorID, FILTER_AC_WIDE_NOTCH, false);
		SET_SENSOR_PARAMETER(sensorID, FILTER_AC_NARROW_NOTCH, false);
		SET_SENSOR_PARAMETER(sensorID, FILTER_DC_ADAPTIVE, 0.0);
	}

	transmitterID = 0;
	{
		// initialize a structure of angles
		DOUBLE_ANGLES_RECORD anglesRecord = { 0, 0, 0 };
		SET_TRANSMITTER_PARAMETER(transmitterID, REFERENCE_FRAME, anglesRecord);
	}
	SET_TRANSMITTER_PARAMETER(transmitterID, XYZ_REFERENCE_FRAME, false);
#endif

	runTimer.start();
}

void magTrackerThread::CheckTrackerPoses()
{
#ifdef MAGTRACKER
	// change to <= 1 if we want to use both trackers
	for (int tracker = 0; tracker < NUM_TRACKERS; tracker = tracker + 1)
	{
		double posScale = 1000.0;
		double depthOffset = 130;
		double heightOffset = 0;
		double horizontalOffset = 0; // -100;

		chai3d::cTransform returnTransform;
		chai3d::cVector3d returnVec;
		chai3d::cMatrix3d returnMatrix;
		chai3d::cMatrix3d initialMatrix;
		chai3d::cVector3d initialVec;
		double x, y, z;

		// test the reading of the magnetic tracker
		errorCode = GetAsynchronousRecord(tracker, &record, sizeof(record));
		if (errorCode != BIRD_ERROR_SUCCESS) { errorHandler(errorCode); }
		// get the status of the last data record
		// only report the data if everything is okay
		x = (record.x - depthOffset) / posScale;
		y = (record.y - horizontalOffset) / posScale;
		z = record.z / posScale;	// (record.z - heightOffset) / posScale;
		initialVec.set(x, y, z);		// no transformation
		
		//cout << "X: " << x << "      Y: " << y << "     Z: " << z << endl;

		m_magTrackerLock.acquire();

		initialMatrix.set(record.s[0][0], record.s[0][1], record.s[0][2],
			record.s[1][0], record.s[1][1], record.s[1][2],
			record.s[2][0], record.s[2][1], record.s[2][2]);
		//returnMatrix.trans();
		//returnMatrix.rotateAboutLocalAxisDeg(1, 0, 0, 180); // for mag tracker chord facing us instead of base box

		cMatrix3d trackerInWorldFrame(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
		trackerInWorldFrame.mulr(initialMatrix, returnMatrix);		// rotate the returnMatrix to the Chai3d coordinate frame
		trackerInWorldFrame.mulr(initialVec, returnVec);			// rotate the position into the Chai3d coordinate frame	

		returnTransform.set(returnVec, returnMatrix);
			// Pass information to chaiDevice for use in the haptics thread
			((chai3d::gripperChaiDevice *)(m_chaiMagDevice->get()))->poseCache = returnTransform;

			m_magTrackerLock.release();

	}

#endif
}

void magTrackerThread::pairWithHapticsThread(chai3d::cGenericHapticDevicePtr *a_chaiMagDevice) {
	m_chaiMagDevice = a_chaiMagDevice;

}

