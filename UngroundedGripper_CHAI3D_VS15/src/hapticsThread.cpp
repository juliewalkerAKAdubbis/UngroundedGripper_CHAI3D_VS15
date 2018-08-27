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
	m_worldLock.acquire();
	setUpWorld();
	setUpHapticDevice();
	setUpODEWorld();
	m_worldLock.release();
	//addObjects();
	
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

int hapticsThread::setUpODEWorld(void) {

	//-----------------------------------------------------------------------
	// CREATE ODE WORLD AND OBJECTS
	//-----------------------------------------------------------------------

	//////////////////////////////////////////////////////////////////////////
	// ODE WORLD
	//////////////////////////////////////////////////////////////////////////

	// stiffness properties
	double maxStiffness = 100; // hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	// create an ODE world to simulate dynamic bodies
	ODEWorld = new cODEWorld(world);

	// add ODE world as a node inside world
	world->addChild(ODEWorld);

	// set some gravity
	ODEWorld->setGravity(cVector3d(0.00, 0.00, -9.81));

	// define damping properties
	ODEWorld->setAngularDamping(0.02);
	ODEWorld->setLinearDamping(0.007);


	//////////////////////////////////////////////////////////////////////////
	// 3 ODE BLOCKS
	//////////////////////////////////////////////////////////////////////////

	// create a new ODE object that is automatically added to the ODE world
	ODEBody0 = new cODEGenericBody(ODEWorld);
	ODEBody1 = new cODEGenericBody(ODEWorld);
	ODEBody2 = new cODEGenericBody(ODEWorld);

	// create a virtual mesh  that will be used for the geometry representation of the dynamic body
	cMesh* object0 = new cMesh();
	cMesh* object1 = new cMesh();
	cMesh* object2 = new cMesh();

	// create a cube mesh
	double size = 0.20;
	cCreateBox(object0, size, size, 3*size);
	object0->createAABBCollisionDetector(toolRadius);

	cCreateBox(object1, size, size, 4*size);
	object1->createAABBCollisionDetector(toolRadius);

	cCreateBox(object2, size, size, 3*size);
	object2->createAABBCollisionDetector(toolRadius);

	// define some material properties for each cube
	cMaterial mat0, mat1, mat2;
	mat0.setRedIndian();
	mat0.setStiffness(0.3 * maxStiffness);
	mat0.setDynamicFriction(0.6);
	mat0.setStaticFriction(0.6);
	object0->setMaterial(mat0);

	mat1.setBlueRoyal();
	mat1.setStiffness(0.3 * maxStiffness);
	mat1.setDynamicFriction(0.6);
	mat1.setStaticFriction(0.6);
	object1->setMaterial(mat1);

	mat2.setGreenDarkSea();
	mat2.setStiffness(0.3 * maxStiffness);
	mat2.setDynamicFriction(0.6);
	mat2.setStaticFriction(0.6);
	object2->setMaterial(mat2);

	// add mesh to ODE object
	ODEBody0->setImageModel(object0);
	ODEBody1->setImageModel(object1);
	ODEBody2->setImageModel(object2);

	// create a dynamic model of the ODE object. Here we decide to use a box just like
	// the object mesh we just defined
	ODEBody0->createDynamicBox(size, size, size);
	ODEBody1->createDynamicBox(size, size, size);
	ODEBody2->createDynamicBox(size, size, size);

	// define some mass properties for each cube
	ODEBody0->setMass(0.05);
	ODEBody1->setMass(0.05);
	ODEBody2->setMass(0.05);

	// set position of each cube
	ODEBody0->setLocalPos(0.0, -0.6, -0.5);
	ODEBody1->setLocalPos(0.0, 0.6, -0.5);
	ODEBody2->setLocalPos(0.0, 0.0, -0.5);

	// rotate central cube 45 degrees around z-axis
	ODEBody0->rotateAboutGlobalAxisDeg(0, 0, 1, 45);


	//////////////////////////////////////////////////////////////////////////
	// 6 ODE INVISIBLE WALLS
	//////////////////////////////////////////////////////////////////////////

	// we create 6 static walls to contains the 3 cubes within a limited workspace
	ODEGPlane0 = new cODEGenericBody(ODEWorld);
	ODEGPlane1 = new cODEGenericBody(ODEWorld);
	ODEGPlane2 = new cODEGenericBody(ODEWorld);
	ODEGPlane3 = new cODEGenericBody(ODEWorld);
	ODEGPlane4 = new cODEGenericBody(ODEWorld);
	ODEGPlane5 = new cODEGenericBody(ODEWorld);

	int w = 1.0;
	ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, 2.0 * w), cVector3d(0.0, 0.0, -1.0));
	ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -w), cVector3d(0.0, 0.0, 1.0));
	ODEGPlane2->createStaticPlane(cVector3d(0.0, w, 0.0), cVector3d(0.0, -1.0, 0.0));
	ODEGPlane3->createStaticPlane(cVector3d(0.0, -w, 0.0), cVector3d(0.0, 1.0, 0.0));
	ODEGPlane4->createStaticPlane(cVector3d(w, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0));
	ODEGPlane5->createStaticPlane(cVector3d(-0.8 * w, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));


	//////////////////////////////////////////////////////////////////////////
	// GROUND
	//////////////////////////////////////////////////////////////////////////

	// create a mesh that represents the ground
	cMesh* ground = new cMesh();
	ODEWorld->addChild(ground);

	// create a plane
	double groundSize = 3.0;
	cCreatePlane(ground, groundSize, groundSize);

	// position ground in world where the invisible ODE plane is located (ODEGPlane1)
	ground->setLocalPos(0.0, 0.0, -1.0);

	// define some material properties and apply to mesh
	cMaterial matGround;
	matGround.setStiffness(0.3 * maxStiffness);
	matGround.setDynamicFriction(0.2);
	matGround.setStaticFriction(0.0);
	matGround.setWhite();
	matGround.m_emission.setGrayLevel(0.3);
	ground->setMaterial(matGround);

	// setup collision detector
	ground->createAABBCollisionDetector(toolRadius);

	return 0;
}
int hapticsThread::setUpWorld(void) {

	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	world = new cWorld();					// create a new world.
	world->m_backgroundColor.setWhite();	// set the background color of the environment
	camera = new cCamera(world);			// create a camera and insert it into the virtual world
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(2.0, 0.0, 0.2),   // camera position (eye)
		cVector3d(0.0, 0.0, -0.5),			// lookat position (target)
		cVector3d(0.0, 0.0, 1.0));			// direction of the "up" vector

											// set the near and far clipping planes of the camera
											// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);
	camera->setStereoMode(stereoMode);				// set stereo mode
	camera->setStereoEyeSeparation(0.02);			// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoFocalLength(2.0);
	camera->setMirrorVertical(mirroredDisplay);		// set vertical mirrored display mode

	light = new cSpotLight(world);			// create a light source
	camera->addChild(light);				// attach light to camera
	light->setEnabled(true);				// enable light source
	light->setLocalPos(0.0, 0.0, 3.0);		// position the light source
	light->setDir(0.0, 0.0, -1.0);			// define the direction of the light beam
	light->setSpotExponent(0.0);			// set uniform concentration level of light 
	light->setShadowMapEnabled(true);		// enable this light source to generate shadows
	light->m_shadowMap->setQualityLow();			// set the resolution of the shadow map
	//light->m_shadowMap->setQualityMedium();
	light->setCutOffAngleDeg(45);			// set light cone half angle

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
	tool = new cToolGripper(world);					// create a 3D tool and add it to the world
	world->addChild(tool);
	
	tool->setHapticDevice(chaiMagDevice);		// connect the haptic device to the tool
	toolRadius = 0.05;							// define the radius of the tool (sphere)
	tool->setRadius(toolRadius);				// define a radius for the tool
	tool->setShowContactPoints(false, false);		// hide the device sphere. only show proxy.
	tool->setShowFrame(false);
	tool->setFrameSize(0.5);
	tool->setWorkspaceRadius(1.0);		// map the physical workspace of the haptic device to a larger virtual workspace.


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


	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// initialize tool by connecting to haptic device
	tool->start();


	//tool->m_hapticPointFinger->m_sphereProxy->addChild(m_curSphere0);
	//tool->m_hapticPointThumb->m_sphereProxy->addChild(m_curSphere1);
	tool->setShowEnabled(true, true);
	tool->m_hapticPointFinger->setShow(true, false);
	tool->m_hapticPointThumb->setShow(true, false);


	loadFingerMeshes();
	return 0;
}


int hapticsThread::addObjects(void) {
	//--------------------------------------------------------------------------
	// CREATE OBJECT
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// stiffness properties
	//double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
	double maxStiffness = 100;					// --------------------------------------- TO DO -------------------------------------------
												
												
												
												

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
	finger->setFrameSize(0.5);			
	finger->setLocalPos(0.0, 0.0, 0.0);

	thumb = new chai3d::cMultiMesh(); //create the thumb
	world->addChild(thumb);
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
	finger->setShowEnabled(false);
	finger->setUseVertexColors(true);
	chai3d::cColorf fingerColor;
	fingerColor.setBrownSandy();
	finger->setVertexColor(fingerColor);
	finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
	finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	finger->m_material->m_specular.set(1.0, 1.0, 1.0);
	finger->setUseMaterial(true);
	finger->setHapticEnabled(false);
	finger->setShowFrame(false);

	// set params for thumb
	thumb->scale(7);
	thumb->setShowEnabled(false);
	thumb->setUseVertexColors(true);
	chai3d::cColorf thumbColor;
	thumbColor.setBrownSandy();
	thumb->setVertexColor(thumbColor);
	thumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
	thumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	thumb->m_material->m_specular.set(1.0, 1.0, 1.0);
	thumb->setUseMaterial(true);
	thumb->setHapticEnabled(false);
	thumb->setShowFrame(false);


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

		m_runLock.acquire();

		/////////////////////////////////////////////////////////////////////
		// HAPTIC FORCE COMPUTATION
		/////////////////////////////////////////////////////////////////////

		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position and orientation of tool
		tool->updateFromDevice();
		
		// compute interaction forces
		tool->computeInteractionForces();


		


		/////////////////////////////////////////////////////////////////////
		// SEND FORCES TO HAPTIC GRIPPER
		/////////////////////////////////////////////////////////////////////
		if (!keyboardCues) {

		m_gripperForce = tool->getDeviceLocalForce();
		m_gripperTorque = tool->getDeviceLocalTorque();
		m_gripperGripForce = tool->getGripperForce();

		m_gripperRot = tool->getLocalRot();

			// get global coordinate forces and convert to local frame
			m_gripperRot.mulr(tool->m_hapticPointThumb->getLastComputedForce(), m_thumbForce);
			m_gripperRot.mulr(tool->m_hapticPointFinger->getLastComputedForce(), m_fingerForce);
		}


		// send to gripper object
		m_runLock.acquire();
		m_gripper->setForcesAndTorques(m_gripperForce, m_gripperTorque, m_gripperGripForce, m_thumbForce, m_fingerForce);
		m_runLock.acquire();
	

		/////////////////////////////////////////////////////////////////////
		// DYNAMIC SIMULATION
		/////////////////////////////////////////////////////////////////////

	
		// https://github.com/aleeper/ros_haptics/blob/master/chaifork3/modules/ODE/examples/GLUT/40-ODE-cube/40-ODE-cubic.cpp
		// for each interaction point of the tool we look for any contact events
		// with the environment and apply forces accordingly
		int numInteractionPoints = tool->getNumHapticPoints();
		for (int i = 0; i<numInteractionPoints; i++)
		{
			// get pointer to next interaction point of tool
			cHapticPoint* interactionPoint = tool->getHapticPoint(i);

			// check all contact points
			int numContacts = interactionPoint->getNumCollisionEvents();
			for (int i = 0; i<numContacts; i++)
			{
				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

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

		// update simulation
		ODEWorld->updateDynamics(timeInterval);
		m_runLock.release();

		// Output to motors
		m_gripper->motorLoop();

	}
		

	// exit haptics thread
	simulationFinished = true;
	m_gripper->disconnect();
}


//------------------------------------------------------------------------------

void hapticsThread::updateGraphics(void)
{	/////////////////////////////////////////////////////////////////////
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
	// UPDATE FINGER GRAPHICS
	/////////////////////////////////////////////////////////////////////

	finger->setLocalPos(tool->m_hapticPointFinger->getGlobalPosProxy());
	finger->setLocalRot(tool->getDeviceLocalRot());
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), 180);
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 90);
	//finger->rotateAboutLocalAxisDeg(cVector3d(-1, 0, 0), (tool->getGripperAngleDeg())/2);

	thumb->setLocalPos(tool->m_hapticPointThumb->getGlobalPosProxy());
	thumb->setLocalRot(tool->getDeviceLocalRot());
	thumb->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), -90);
	//thumb->rotateAboutLocalAxisDeg(cVector3d(1, 0, 0), -(tool->getGripperAngleDeg()) / 2);

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
