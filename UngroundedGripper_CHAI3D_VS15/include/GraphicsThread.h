#pragma once

#include <GLFW/glfw3.h>
#include "chai3d.h"
#include <ctime>
#include <cstdlib>
#include "Gripper.h"

using namespace std;
using namespace chai3d;


class Graphics
{
public: 

	Gripper* gripper;		// pointer to gripper 

	//------------------------------------------------------------------------------
	// GENERAL SETTINGS
	//------------------------------------------------------------------------------
	cStereoMode stereoMode = C_STEREO_DISABLED;
	// fullscreen mode
	bool fullscreen = false;
	// mirrored display
	bool mirroredDisplay = false;

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


	// a virtual object
	cMultiMesh* object;



	chai3d::cFrequencyCounter graphicRate;  // counter for graphics updates
	chai3d::cFrequencyCounter hapticRate;   // counter for haptics updates

	chai3d::cWorld* world;                  // CHAI world
	chai3d::cCamera* camera;                // camera to render the world
	chai3d::cSpotLight* light;       // light to illuminate the world
									 // a handle to window display context
	GLFWwindow* window = NULL;

	clock_t* m_timer;						// timer for graphics updates
										 

	int width;                              // width of view
	int height;                             // height of view
	int mouseX;                             // cursor X-position
	int mouseY;                             // cursor Y-position

	cFontPtr font;				// a font for rendering text
	cLabel* labelRates;			// a label to display the rate [Hz] at which the simulation is running
											
	cHapticDeviceHandler* handler;			// a haptic device handler
	cGenericHapticDevicePtr hapticDevice;	// a pointer to the current haptic device
	cToolCursor* tool;						// a virtual tool representing the haptic device in the scene

											
	int swapInterval = 1;			// swap interval for the display context (vertical synchronization)





	bool start();
	void stop();
	void* hapticThread();
	int initialize(void);
	int setUpWorld(void);
	int setUpHapticDevice(void);
	void pairWithGripper(Gripper * a_gripper);
	void setUpWidgets(void);
	void graphicsLoop(void);

protected:
	
	// callback when the window display is resized
	void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

	// callback when an error GLFW occurs
	void errorCallback(int error, const char* a_description);

	// this function renders the scene
	void updateGraphics(void);
	
	void drawGripper();
	int Graphics::addObject(void);
};

