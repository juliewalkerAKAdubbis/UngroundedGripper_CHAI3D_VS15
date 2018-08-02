#include "hapticsThread.h"



void hapticsThread::pairWithGripper(gripper *a_gripper) {
	m_gripper = a_gripper;

}

int hapticsThread::initialize( void ) {
	int errors = 0;
	
	// create Chai device for magnetic tracker
	chaiMagDevice = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::gripperChaiDevice( 0)));

	initializeChai3dStuff();
	setUpWorld();
	setUpHapticDevice();
	addObjects();
	
	return errors;
}

// initialization sub-functions
int hapticsThread::initializeChai3dStuff(void) {
	
	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

		// initialize GLFW library
		if (!glfwInit())
		{
			cout << "failed initialization" << endl;
			cSleepMs(1000);
			return 1;
		}


		// set error callback
		glfwSetErrorCallback(errorCallback);

		// compute desired size of window
		const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		int w = 0.8 * mode->height;
		int h = 0.5 * mode->height;
		int x = 0.5 * (mode->width - w);
		int y = 0.5 * (mode->height - h);



		// set OpenGL version
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

		if (stereoMode == C_STEREO_ACTIVE)								// set active stereo mode
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

		glfwGetWindowSize(window, &width, &height);						// get width and height of window
		glfwSetWindowPos(window, x, y);									// set position of window
		glfwMakeContextCurrent(window);									// set current display context
		glfwSwapInterval(swapInterval);									// sets the swap interval for the current display context




#ifdef GLEW_VERSION
		// initialize GLEW library
		if (glewInit() != GLEW_OK)
		{
			cout << "failed to initialize GLEW library" << endl;
			glfwTerminate();
			return 1;
		}
#endif


		//hapticsThread::setUpWorld();			// world - camera - lighting

		return 0;
}

int hapticsThread::setUpWorld(void) {

	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

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
	light->setShadowMapEnabled(false);		// disable this light source to generate shadows
	//light->m_shadowMap->setQualityLow();			// set the resolution of the shadow map
	//light->m_shadowMap->setQualityMedium();
	light->setCutOffAngleDeg(20);			// set light cone half angle

	return 0;
}

int hapticsThread::setUpHapticDevice(void) {


	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	//handler = new cHapticDeviceHandler();			// create a haptic device handler
	//handler->getDevice(hapticDevice, 0);				// get access to the first available haptic device
	//cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();		// retrieve information about the current haptic device
	//hapticDevice->setEnableGripperUserSwitch(true);			// if the haptic devices carries a gripper, enable it to behave like a user switch
	chaiMagDevice->setEnableGripperUserSwitch(true);
	tool = new cToolCursor(world);			// create a 3D tool and add it to the world
	world->addChild(tool);
	
	//tool->setHapticDevice(hapticDevice);
	tool->setHapticDevice(chaiMagDevice);		// connect the haptic device to the tool
	toolRadius = 0.05;							// define the radius of the tool (sphere)
	tool->setRadius(toolRadius);				// define a radius for the tool
	tool->setShowContactPoints(true, false);		// hide the device sphere. only show proxy.

	// ADD SECOND TOOL FOR OTHER FINGER						//---------------------------------- TO DO ------------------------------------------------------//

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

	// Can use this to show frames on tool if so desired
	//create a sphere to represent the tool
	m_curSphere0 = new chai3d::cShapeSphere(toolRadius);
	world->addChild(m_curSphere0);
	m_curSphere0->m_material->setRed(); // setGrayDarkSlate();
	m_curSphere0->setShowFrame(false);
	m_curSphere0->setFrameSize(0.05);
	m_curSphere0->setTransparencyLevel(0);
	cVector3d startPos = { 0.5, 0.5, 0.5 };
	m_curSphere0->setLocalPos(startPos);

	return 0;
}


int hapticsThread::addObjects(void) {
	//--------------------------------------------------------------------------
	// CREATE OBJECT
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// stiffness properties
	//double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
	double maxStiffness = 1;					// --------------------------------------- TO DO -------------------------------------------
												
												
												
												

	//// create a cylinder
	//cylinder = new cShapeCylinder(0.25, 0.25, 0.2);
	//world->addChild(cylinder);

	//// set position and orientation
	//cylinder->setLocalPos(0.4, 0.4, 0.0);
	//cylinder->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90);

	//// set material color
	//cylinder->m_material->setBlueCornflower();

	//// create haptic effect and set properties
	//cylinder->createEffectSurface();
	//cylinder->m_material->setStiffness(0.8 * maxStiffness);




	// create a virtual mesh
	object = new cMultiMesh();

	// add object to world
	world->addChild(object);

	// set the position of the object at the center of the world
	object->setLocalPos(0.3, 0.0, 0.0);

	// rotate the object 90 degrees
	//object->rotateAboutGlobalAxisDeg(cVector3d(0, 0, 1), 90);

	// load an object file
	bool fileload;
	fileload = object->loadFromFile("../../external/chai3d-3.2.0/bin/resources/models/tooth/tooth.obj");
	if (!fileload)
	{
#if defined(_MSVC)
		fileload = object->loadFromFile("../../../bin/resources/models/tooth/tooth.obj");
#endif
	}
	if (!fileload)
	{
		cout << "Error - 3D Model failed to load correctly." << endl;
		//close();
		return (-1);
	}

	object->computeBoundaryBox(true);		// compute a boundary box
	double size = cSub(object->getBoundaryMax(), object->getBoundaryMin()).length();		// get dimensions of object

	if (size > 0)			// resize object to screen
	{
		object->scale(2.0 * tool->getWorkspaceRadius() / size);
	}

	object->createAABBCollisionDetector(toolRadius);		// compute collision detection algorithm

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
		//close();
		return (-1);
	}

	texture->setSphericalMappingEnabled(true);		// enable spherical mapping
	object->setTexture(texture, true);				// assign texture to object
	object->setUseTexture(true, true);				// enable texture mapping
	object->setUseCulling(false, true);				// disable culling
	object->setStiffness(maxStiffness, true);		// define a default stiffness for the object
	object->setFriction(0.1, 0.2, true);			// define some haptic friction properties

	return 0;
};

void hapticsThread::setUpWidgets(void) {
	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	font = NEW_CFONTCALIBRI20();		//create a font

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(labelRates);

}




// main thread loop
void hapticsThread::updateHaptics(void)
{
	// angular velocity
	cVector3d angVel(0, 0, 0);

	// reset clock
	cPrecisionClock clock;
	clock.reset();

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// SIMULATION TIME
		/////////////////////////////////////////////////////////////////////

		// stop the simulation clock
		clock.stop();

		// read the time increment in seconds
		double timeInterval = clock.getCurrentTimeSeconds();

		// restart the simulation clock
		clock.reset();
		clock.start();

		// signal frequency counter
		hapticRate.signal(1);


		/////////////////////////////////////////////////////////////////////
		// HAPTIC FORCE COMPUTATION
		/////////////////////////////////////////////////////////////////////

		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position and orientation of tool
		tool->updateFromDevice();
		position = tool->m_hapticPoint->getGlobalPosGoal(); // get position and rotation of the haptic point (and delta mechanism) (already transformed from magTracker)
		chaiMagDevice->getRotation(rotation);
		m_curSphere0->setLocalPos(position); // set the sphere visual representation to match
		m_curSphere0->setLocalRot(rotation);
		//cout << position << endl;

		// compute interaction forces
		tool->computeInteractionForces();

		// send forces to haptic device
		tool->applyToDevice();
		cout << tool->m_interactionNormal << endl;

		/////////////////////////////////////////////////////////////////////
		// DYNAMIC SIMULATION
		/////////////////////////////////////////////////////////////////////

		// get position of cursor in global coordinates
		cVector3d toolPos = tool->getDeviceGlobalPos();

		// get position of object in global coordinates
		cVector3d objectPos = object->getGlobalPos();

		// compute a vector from the center of mass of the object (point of rotation) to the tool
		cVector3d v = cSub(toolPos, objectPos);

		// compute angular acceleration based on the interaction forces
		// between the tool and the object
		cVector3d angAcc(0, 0, 0);
		if (v.length() > 0.0)
		{
			// get the last force applied to the cursor in global coordinates
			// we negate the result to obtain the opposite force that is applied on the
			// object
			cVector3d toolForce = -tool->getDeviceGlobalForce();

			// compute the effective force that contributes to rotating the object.
			cVector3d force = toolForce - cProject(toolForce, v);

			// compute the resulting torque
			cVector3d torque = cMul(v.length(), cCross(cNormalize(v), force));

			// compute a torque to restore the face to its original position
			cVector3d dirFace = object->getLocalRot().getCol0();
			cVector3d dirTorque = cCross(dirFace, cVector3d(0, 1, 0));
			dirTorque.mul(3.0);
			torque.add(dirTorque);

			cVector3d upFace = object->getLocalRot().getCol2();
			cVector3d upTorque = cCross(upFace, cVector3d(0, 0, 1));
			dirTorque.mul(3.0);
			torque.add(upTorque);

			// update rotational acceleration
			const double INERTIA = 0.4;
			angAcc = (1.0 / INERTIA) * torque;
		}

		// update rotational velocity
		angVel.add(timeInterval * angAcc);

		// set a threshold on the rotational velocity term
		const double MAX_ANG_VEL = 10.0;
		double vel = angVel.length();
		if (vel > MAX_ANG_VEL)
		{
			angVel.mul(MAX_ANG_VEL / vel);
		}

		// add some damping too
		const double DAMPING = 0.1;
		angVel.mul(1.0 - DAMPING * timeInterval);

		// if user switch is pressed, set velocity to zero
		if (tool->getUserSwitch(0) == 1)
		{
			angVel.zero();
		}

		// compute the next rotation configuration of the object
		if (angVel.length() > C_SMALL)
		{
			object->rotateAboutGlobalAxisRad(cNormalize(angVel), timeInterval * angVel.length());
		}

		//updateGraphics();

	}

	// exit haptics thread
	simulationFinished = true;
}


//------------------------------------------------------------------------------

void hapticsThread::updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	//// update haptic and graphic rate data
	//labelRates->setText(cStr(graphicRate.getFrequency(), 0) + " Hz / " +
	//	cStr(hapticRate.getFrequency(), 0) + " Hz");

	//// update position of label
	//labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	
	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	cout << " Update Graphics " << endl;

	//m_worldLock.acquire();
	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world	
	camera->renderView(width, height);                           

	// wait until all GL commands are completed
	glFinish();

	//m_worldLock.release();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}


//------------------------------------------------------------------------------
// Helpers
//------------------------------------------------------------------------------

bool hapticsThread::checkSimulationStatus(void) {
	if (simulationFinished) {
		return true;
	}
	else {
		return false;	}
}


void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}
