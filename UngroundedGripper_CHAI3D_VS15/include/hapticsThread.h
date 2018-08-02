#pragma once

#include "chai3d.h"
#include "motorcontrol.h"
#include "gripper.h"
#include "gripperChaiDevice.h"
#include "magtracker.h"
#include "Windows.h"
#include <cmath>
#include <array>
#include <stdio.h>
#include <time.h>
#include <GLFW/glfw3.h>

using namespace chai3d;
using namespace std;


#define RANGE_10V 0x00 // Range code for ADC ±10V range.
#define RANGE_5V 0x10 // Range code for ADC ±5V range.
#define EOPL 0x80 // ADC end-of-poll-list marker.
#define CHANMASK 0x0F // ADC channel number mask.

// non member prototypes
void errorCallback(int a_error, const char* a_description);


class hapticsThread
{
public:

	// initialization functions
	int initialize(void);
	int initializeChai3dStuff(void);
	int setUpWorld(void);
	int setUpHapticDevice(void);
	int addObjects(void);
	void setUpWidgets(void);

	// running functions
	void updateHaptics(void);
	void updateGraphics(void);
	


	//------------------------------------------------------------------------------
	// GENERAL SETTINGS
	//------------------------------------------------------------------------------
	cStereoMode stereoMode = C_STEREO_DISABLED;
	// fullscreen mode
	bool fullscreen = false;
	// mirrored display
	bool mirroredDisplay = false;

	GLFWwindow* window = NULL;


	// rendering option
	bool showTexture = true;
	bool showNormals = false;
	bool showWireMode = false;

	//------------------------------------------------------------------------------
	// THREADING
	//------------------------------------------------------------------------------

	chai3d::cThread m_thread;
	chai3d::cMutex m_worldLock;
	chai3d::cMutex m_runLock;

//simulation status
	bool simulationRunning = true;
	bool simulationFinished = false;
	bool checkSimulationStatus(void);


	chai3d::cFrequencyCounter graphicRate;  // counter for graphics updates
	chai3d::cFrequencyCounter hapticRate;   // counter for haptics updates


											// a virtual object
	cMultiMesh* object;
	cShapeCylinder* cylinder;
	cShapeSphere* m_curSphere0;

	chai3d::cWorld* world;                  // CHAI world
	chai3d::cCamera* camera;                // camera to render the world
	chai3d::cSpotLight* light;       // light to illuminate the world
									 // a handle to window display context

	clock_t* m_timer;						// timer for graphics updates


	int width;                              // width of view
	int height;                             // height of view
	int mouseX;                             // cursor X-position
	int mouseY;                             // cursor Y-position

	cFontPtr font;				// a font for rendering text
	cLabel* labelRates;			// a label to display the rate [Hz] at which the simulation is running


	chai3d::cGenericHapticDevicePtr chaiMagDevice;

	// tracker rotation variables
	chai3d::cVector3d position; chai3d::cMatrix3d rotation;
	//chai3d::cMatrix3d fingerRotation0; chai3d::cMatrix3d deviceRotation0;

	cHapticDeviceHandler* handler;			// a haptic device handler
	cGenericHapticDevicePtr hapticDevice;	// a pointer to the current haptic device
	cToolCursor* tool;						// a virtual tool representing the haptic device in the scene
	gripper* m_gripper;						// the member gripper instantiation
	void hapticsThread::pairWithGripper(gripper *a_gripper);

	double toolRadius;

	int swapInterval = 1;			// swap interval for the display context (vertical synchronization)

	//protected:


};