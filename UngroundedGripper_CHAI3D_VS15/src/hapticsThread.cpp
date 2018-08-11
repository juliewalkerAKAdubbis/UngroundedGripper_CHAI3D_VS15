#include "hapticsThread.h"


// call in main before starting thread 
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
	
	// set output to zero to start
	m_gripperForce = { 0.0, 0.0, 0.0 };
	m_gripperTorque = { 0.0, 0.0, 0.0 };
	m_gripperGripForce = 0.0;

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
	tool = new cToolGripper(world);			// create a 3D tool and add it to the world
	world->addChild(tool);
	

	//tool->setHapticDevice(hapticDevice);
	tool->setHapticDevice(chaiMagDevice);		// connect the haptic device to the tool
	toolRadius = 0.05;							// define the radius of the tool (sphere)
	tool->setRadius(toolRadius);				// define a radius for the tool
	tool->setShowContactPoints(false, false);		// hide the device sphere. only show proxy.
	tool->setShowFrame(false);
	tool->setFrameSize(0.5);


	// ADD SECOND TOOL FOR OTHER FINGER						//---------------------------------- TO DO ------------------------------------------------------//
	//tool2 = new cToolCursor(world);
	//world->addChild(tool2);
	//tool2->setHapticDevice(chaiMagDevice);
	//tool2->setRadius(toolRadius);
	//tool2->setShowFrame(true);
	//tool2->setFrameSize(0.5);

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

	//create a sphere to represent each side of the gripper tool
	m_curSphere0 = new chai3d::cShapeSphere(1.1*toolRadius);
	m_curSphere0->m_material->setRed(); // setGrayDarkSlate();
	m_curSphere0->setTransparencyLevel(0);
	m_curSphere0->setShowFrame(false);			// Can use this to show frames on tool if so desired
	m_curSphere0->setFrameSize(0.1);

	m_curSphere1 = new chai3d::cShapeSphere(1.1*toolRadius);
	m_curSphere1->m_material->setGreenLawn(); // setGrayDarkSlate();
	m_curSphere1->setTransparencyLevel(0);
	m_curSphere1->setShowFrame(false);			// Can use this to show frames on tool if so desired
	m_curSphere1->setFrameSize(0.1);

	//m_gripperBase = new chai3d::cShapeBox(.1, .2, .1);
	////world->addChild(m_gripperBase);
	////cVector3d startPos = { 0.5, 0.5, 0.5 };
	////m_gripperBase->setLocalPos(startPos);
	//m_gripperBase->m_material->setBlue();
	//m_gripperBase->setTransparencyLevel(0);

	//tool->m_hapticPointFinger->m_sphereProxy->addChild(m_curSphere0);
	//tool->m_hapticPointThumb->m_sphereProxy->addChild(m_curSphere1);
	tool->setShowEnabled(true, true);
	tool->m_hapticPointFinger->setShow(false, false);
	tool->m_hapticPointThumb->setShow(false, false);
	//// create a small white line that will be enabled every time the operator
	//// grasps an object. The line indicated the connection between the
	//// position of the tool and the grasp position on the object
	//graspLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	//world->addChild(graspLine);
	//graspLine->m_colorPointA.set(1.0, 1.0, 1.0);
	//graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
	//graspLine->setShowEnabled(false);

	loadFingerMeshes();
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


//--------------------------------------------------------------------------
// GROUND
//--------------------------------------------------------------------------
	ground = new cMesh();
	world->addChild(ground);
	// create 4 vertices (one at each corner)
	double groundSize = 2.0;
	int vertices0 = ground->newVertex(-groundSize, -groundSize, 0.0);
	int vertices1 = ground->newVertex(groundSize, -groundSize, 0.0);
	int vertices2 = ground->newVertex(groundSize, groundSize, 0.0);
	int vertices3 = ground->newVertex(-groundSize, groundSize, 0.0);

	// compose surface with 2 triangles
	ground->newTriangle(vertices0, vertices1, vertices2);
	ground->newTriangle(vertices0, vertices2, vertices3);

	// compute surface normals
	ground->computeAllNormals();

	// position ground at the right level
	ground->setLocalPos(0.0, 0.0, -0.9);

	// define some material properties and apply to mesh
	cMaterial matGround;
	matGround.setDynamicFriction(0.7);
	matGround.setStaticFriction(1.0);
	matGround.m_ambient.set(0.0, 0.0, 0.0);
	matGround.m_diffuse.set(0.0, 0.0, 0.0);
	matGround.m_specular.set(0.0, 0.0, 0.0);
	ground->setMaterial(matGround);

	// enable and set transparency level of ground
	ground->setTransparencyLevel(0.1);
	ground->setUseTransparency(true);

	// setup collision detector
	ground->createAABBCollisionDetector(toolRadius);

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

	labelForce = new cLabel(font);
	labelForce->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(labelForce);

}

void hapticsThread::loadFingerMeshes(void) {
	//--------------------------------------------------------------------------
	// FINGER MESHES
	//--------------------------------------------------------------------------
	finger = new chai3d::cMultiMesh(); // create the finger
	world->addChild(finger);	
	finger->setShowFrame(true);			// show axes if desired
	finger->setFrameSize(0.5);			
	finger->setLocalPos(0.0, 0.0, 0.0);

	thumb = new chai3d::cMultiMesh(); //create the thumb
	world->addChild(thumb);
	thumb->setShowFrame(true);		// show axes if desired
	thumb->setFrameSize(0.5);
	thumb->setLocalPos(0, 0, 0);

	// load an object file
	if (cLoadFileOBJ(finger, "../Resources/FingerModel.obj")) {
		cout << "Finger file loaded." << endl;
	}
	else {
		cout << "Failed to load finger model." << endl;
	}
	if (cLoadFileOBJ(thumb, "../Resources/FingerModelThumb.obj")) {
		cout << "Thumb file loaded." << endl;
	}

	// set params for finger
	finger->scale(7);
	finger->setShowEnabled(true);
	finger->setUseVertexColors(true);
	chai3d::cColorf fingerColor;
	fingerColor.setBrownSandy();
	finger->setVertexColor(fingerColor);
	finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
	finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	finger->m_material->m_specular.set(1.0, 1.0, 1.0);
	finger->setUseMaterial(true);
	finger->setHapticEnabled(false);
	finger->setShowFrame(true);

	// set params for thumb
	thumb->scale(7);
	thumb->setShowEnabled(true);
	thumb->setUseVertexColors(true);
	chai3d::cColorf thumbColor;
	thumbColor.setBrownSandy();
	thumb->setVertexColor(thumbColor);
	thumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
	thumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	thumb->m_material->m_specular.set(1.0, 1.0, 1.0);
	thumb->setUseMaterial(true);
	thumb->setHapticEnabled(false);
	thumb->setShowFrame(true);


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
		//position = tool->m_hapticPoint->getGlobalPosGoal(); // get position and rotation of the haptic point (and delta mechanism) (already transformed from magTracker)
		//chaiMagDevice->getRotation(rotation);
		//m_curSphere0->setLocalPos(position); // set the sphere visual representation to match
		//m_curSphere0->setLocalRot(rotation);
		//cout << position << endl;
		
		// compute interaction forces
		tool->computeInteractionForces();

		/*
		// https://github.com/aleeper/ros_haptics/blob/master/chaifork3/modules/ODE/examples/GLUT/40-ODE-cube/40-ODE-cubic.cpp 
		// for each interaction point of the tool we look for any contact events
		// with the environment and apply forces accordingly
		int numInteractionPoints = tool->getNumInteractionPoints();
		for (int i = 0; i<numInteractionPoints; i++)
		{
			// get pointer to next interaction point of tool
			cHapticPoint* interactionPoint = tool->getInteractionPoint(i);

			// check primary contact point if available
			if (interactionPoint->getNumCollisionEvents() > 0)
			{
				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

				// given the mesh object we may be touching, we search for its owner which
				// could be the mesh itself or a multi-mesh object. Once the owner found, we
				// look for the parent that will point to the ODE object itself.
				cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

				// cast to ODE object
				cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

				// if ODE object, we apply interaction forces
				if (ODEobject != NULL)
				{
					ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
						collisionEvent->m_globalPos);
				}
			}
		}
		*/


		/////////////////////////////////////////////////////////////////////
		// UPDATE FINGER GRAPHICS
		/////////////////////////////////////////////////////////////////////
		finger->setLocalPos(tool->m_hapticPointFinger->getLocalPosProxy());
		finger->setLocalRot(tool->getDeviceLocalRot());
		finger->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), 180);
		finger->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 90);
		//thumb->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), tool->getGripperAngleDeg());
		// //finger->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), -80);
		thumb->setLocalPos(tool->m_hapticPointThumb->getLocalPosProxy());
		thumb->setLocalRot(tool->getDeviceLocalRot());
		thumb->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), -90);
		//thumb->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), -(tool->getGripperAngleDeg()));
		
		//tool->m_hapticPointFinger->setShow(true);

		/////////////////////////////////////////////////////////////////////
		// SEND FORCES TO HAPTIC GRIPPER
		/////////////////////////////////////////////////////////////////////
		//tool->applyToDevice();
		//cout << tool->m_interactionNormal << endl;
		m_gripperForce = tool->getDeviceLocalForce();
		m_gripperTorque = tool->getDeviceLocalTorque();
		m_gripperGripForce = tool->getGripperForce();

		m_gripperRot = tool->getLocalRot();

		// get global coordinate forces and convert to local frame
		m_gripperRot.mulr(tool->m_hapticPointThumb->getLastComputedForce(), m_thumbForce);
		m_gripperRot.mulr(tool->m_hapticPointFinger->getLastComputedForce(), m_fingerForce);


		// send to gripper object
		m_runLock.acquire();
		m_gripper->setForcesAndTorques(m_gripperForce, m_gripperTorque, m_gripperGripForce, m_thumbForce, m_fingerForce);
		m_runLock.acquire();
	

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
			cVector3d force = toolForce - cProject(toolForce, v);					// compute the effective force that contributes to rotating the object.
			cVector3d torque = cMul(v.length(), cCross(cNormalize(v), force));					// compute the resulting torque


			// compute a torque to restore the object to its original position
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
	// labelRates->setText(cStr(graphicRate.getFrequency(), 0) + " Hz / " +
	//	cStr(hapticRate.getFrequency(), 0) + " Hz");

	//// update position of label
	//labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	// update force rendered
	//labelForce->setText(cStr(&m_gripperForce.x, 0) + ", " + cStr(&m_gripperForce.y, 0) + ", " + cStr(&m_gripperForce.z, 0) + " N" );
	//labelForce->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	
	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

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
