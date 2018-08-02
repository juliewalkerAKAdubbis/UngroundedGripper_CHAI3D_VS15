/* This file contains the functions for initializing and running the graphics thread with Chai3D */

#include "GraphicsThread.h"


//------------------------------------------------------------------------------

int Graphics::initialize() {

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}


	// set error callback
	glfwSetErrorCallback(Graphics::errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);



	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	if (stereoMode == C_STEREO_ACTIVE)						// set active stereo mode
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);			// create display context

	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	glfwGetWindowSize(window, &width, &height);				// get width and height of window
	glfwSetWindowPos(window, x, y);						// set position of window
	glfwSetKeyCallback(window, keyCallback);			// set key callback
	glfwSetWindowSizeCallback(window, Graphics::windowSizeCallback);			// set resize callback
	glfwMakeContextCurrent(window);						// set current display context
	glfwSwapInterval(swapInterval);						// sets the swap interval for the current display context



	Graphics::setUpWorld();


#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif

	Graphics::setUpHapticDevice();	
	Graphics::addObject();
	Graphics::setUpWidgets();

}

void Graphics::graphicsLoop(void) {
	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	Graphics::windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		Graphics::updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		graphicRate.signal(1);
	}


}
//////////////// INITIALIZATION FUNCTIONS ///////////////////////
int Graphics::setUpWorld(void) {

	world = new cWorld();					// create a new world.
	world->m_backgroundColor.setBlack();	// set the background color of the environment
	camera = new cCamera(world);			// create a camera and insert it into the virtual world
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(3.0, 0.0, 0.6),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),			// Lookat position (target)
		cVector3d(0.0, 0.0, 1.0));			// direction of the (up) vector

											// set the near and far clipping planes of the camera
											// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);
	camera->setStereoMode(stereoMode);			// set stereo mode
	camera->setStereoEyeSeparation(0.03);			// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoFocalLength(3.0);
	camera->setMirrorVertical(mirroredDisplay);			// set vertical mirrored display mode

	light = new cSpotLight(world);			// create a light source
	camera->addChild(light);				// attach light to camera
	light->setEnabled(true);				// enable light source
	light->setLocalPos(0.0, 0.5, 0.0);		// position the light source
	light->setDir(-3.0, -0.5, 0.0);			// define the direction of the light beam
	light->setShadowMapEnabled(true);		// enable this light source to generate shadows


	light->m_shadowMap->setQualityLow();			// set the resolution of the shadow map
													//light->m_shadowMap->setQualityMedium();
	light->setCutOffAngleDeg(20);		// set light cone half angle

	return 0;
}

int Graphics::addObject(void) {
	//--------------------------------------------------------------------------
	// CREATE OBJECT
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// stiffness properties
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	// create a virtual mesh
	object = new cMultiMesh();

	// add object to world
	world->addChild(object);

	// set the position of the object at the center of the world
	object->setLocalPos(0.0, 0.0, 0.0);

	// rotate the object 90 degrees
	object->rotateAboutGlobalAxisDeg(cVector3d(0, 0, 1), 90);

	// load an object file
	bool fileload;
	fileload = object->loadFromFile("../../external/chai3d-3.2.0/bin/resources/models/face/face.obj");
	if (!fileload)
	{
#if defined(_MSVC)
		fileload = object->loadFromFile("../../../bin/resources/models/face/face.obj");
#endif
	}
	if (!fileload)
	{
		cout << "Error - 3D Model failed to load correctly." << endl;
		Graphics::close();
		return (-1);
	}

	// compute a boundary box
	object->computeBoundaryBox(true);

	// get dimensions of object
	double size = cSub(object->getBoundaryMax(), object->getBoundaryMin()).length();

	// resize object to screen
	if (size > 0)
	{
		object->scale(2.0 * tool->getWorkspaceRadius() / size);
	}

	// compute collision detection algorithm
	object->createAABBCollisionDetector(toolRadius);

	cMaterial mat;
	mat.setHapticTriangleSides(true, true);
	object->setMaterial(mat);

	// define some environmental texture mapping
	cTexture2dPtr texture = cTexture2d::create();

	// load texture file
	fileload = texture->loadFromFile("../../external/chai3d-3.2.0/bin/resources/images/chrome.jpg");
	if (!fileload)
	{
#if defined(_MSVC)
		fileload = texture->loadFromFile("../../../bin/resources/images/chrome.jpg");
#endif
	}
	if (!fileload)
	{
		cout << "Error - Texture image failed to load correctly." << endl;
		close();
		return (-1);
	}

	// enable spherical mapping
	texture->setSphericalMappingEnabled(true);

	// assign texture to object
	object->setTexture(texture, true);

	// enable texture mapping
	object->setUseTexture(true, true);

	// disable culling
	object->setUseCulling(false, true);

	// define a default stiffness for the object
	object->setStiffness(maxStiffness, true);

	// define some haptic friction properties
	object->setFriction(0.1, 0.2, true);

};


void Graphics::pairWithGripper(Gripper * a_gripper) {
	gripper = a_gripper;
}

int Graphics::setUpHapticDevice(void) {


	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	handler = new cHapticDeviceHandler();			// create a haptic device handler
	handler->getDevice(hapticDevice, 0);				// get access to the first available haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();		// retrieve information about the current haptic device
	hapticDevice->setEnableGripperUserSwitch(true);			// if the haptic devices carries a gripper, enable it to behave like a user switch
	tool = new cToolCursor(world);			// create a 3D tool and add it to the world
	world->addChild(tool);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// define the radius of the tool (sphere)
	double toolRadius = 0.04;

	// define a radius for the tool
	tool->setRadius(toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(true, false);

	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(true);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// initialize tool by connecting to haptic device
	tool->start();

}


void Graphics::setUpWidgets(void){
// WIDGETS

// create a font
font = NEW_CFONTCALIBRI20();

// create a label to display the haptic and graphic rate of the simulation
labelRates = new cLabel(font);
labelRates->m_fontColor.setWhite();
camera->m_frontLayer->addChild(labelRates);

}


void Graphics::windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;
}

//------------------------------------------------------------------------------

void Graphics::errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}


//------------------------------------------------------------------------------

void Graphics::updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	labelRates->setText(cStr(graphicRate.getFrequency(), 0) + " Hz / " +
		cStr(hapticRate.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

void Graphics::drawGripper(void) {
	cout << "This function doesn't do anything yet. Add code to draw the gripper." << endl;
}
