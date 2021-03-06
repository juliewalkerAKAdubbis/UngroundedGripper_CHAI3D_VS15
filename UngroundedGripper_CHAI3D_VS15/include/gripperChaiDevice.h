#pragma once
/* This file reads in position from the magnetic tracker and treats the
* information as a CHAI device */


//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2014, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Julie Walker, Stanford University
\version   3.0.0 $Rev: 1242 $
*/
//==============================================================================


#define C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "magtracker.h"
#include "gripper.h"
#include <algorithm>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\file       gripperChaiDevice.h

	\brief
	<b> Devices </b> \n
	Custom Haptic Device (Template).
	*/
	//==============================================================================

	//------------------------------------------------------------------------------
	class gripperChaiDevice;
	typedef std::shared_ptr<gripperChaiDevice> gripperChaiDevicePtr;
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\class      gripperChaiDevice
	\ingroup    devices

	\brief
	Interface to custom haptic device (template).

	\details
	gripperChaiDevice provides a basic template which allows to very easily
	interface CHAI3D to your own custom haptic device. \n\n

	Simply follow the 11 commented step in file gripperChaiDevice.cpp
	and complete the code accordingly.
	Depending of the numbers of degrees of freedom of your device, not
	all methods need to be implemented. For instance, if your device
	does not provide any rotation degrees of freedom, simply ignore
	the getRotation() method. Default values will be returned correctly
	if these are not implemented on your device. In the case of rotations
	for instance, the identity matrix is returned.\n\n

	You may also rename this class in which case you will also want to
	customize the haptic device handler to automatically detect your device.
	Please consult method update() of the cHapticDeviceHandler class
	that is located in file CHapticDeviceHandler.cpp .
	Simply see how the haptic device handler already looks for
	device of type gripperChaiDevice.\n\n

	If you are encountering any problems with your implementation, check
	for instance file cDeltaDevices.cpp which implement supports for the
	Force Dimension series of haptic devices. In order to verify the implementation
	use the 01-device example to get started. Example 11-effects is a great
	demo to verify how basic haptic effects may behave with you haptic
	devices. If you do encounter vibrations or instabilities, try reducing
	the maximum stiffness and/or damping values supported by your device.
	(see STEP-1 in file c3dofChaiDevice.cpp).\n

	Make  sure that your device is also communicating fast enough with
	your computer. Ideally the communication period should take less
	than 1 millisecond in order to reach a desired update rate of at least 1000Hz.
	Problems can typically occur when using a slow serial port (RS232) for
	instance.\n
	*/
	//==============================================================================
	class gripperChaiDevice : public cGenericHapticDevice
	{
		//--------------------------------------------------------------------------
		// CONSTRUCTOR & DESTRUCTOR:
		//--------------------------------------------------------------------------

	public:

		//! Constructor of c3dofChaiDevice.
		gripperChaiDevice( unsigned int a_deviceNumber = 0);

		//! Destructor of c3dofChaiDevice.
		virtual ~gripperChaiDevice();

		//! Shared c3dofChaiDevice allocator.
		static gripperChaiDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<gripperChaiDevice>( a_deviceNumber)); }
		

		//--------------------------------------------------------------------------
		// PUBLIC METHODS:
		//--------------------------------------------------------------------------

	public:

		//! Open connection to haptic device.
		virtual bool open();

		//! Close connection to haptic device.
		virtual bool close();

		//! Calibrate haptic device.
		virtual bool calibrate(bool a_forceCalibration = false);

		//! Read the position of the device. Units are meters [m].
		virtual bool getPosition(cVector3d& a_position);

		//! Read the orientation frame of the device handle.
		virtual bool getRotation(cMatrix3d& a_rotation);

		//! Read the gripper angle in radian [rad].
		virtual bool getGripperAngleRad(double& a_angle);

		//! Send a force [N] and a torque [N*m] and gripper force [N] to haptic device.
		virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);

		//! Read status of the switch [__true__ = __ON__ / __false__ = __OFF__].
		virtual bool getUserSwitch(int a_switchIndex, bool& a_status);


		//--------------------------------------------------------------------------
		// PUBLIC STATIC METHODS:
		//--------------------------------------------------------------------------

	public:

		//! Returns the number of devices available from this class of device.
		static unsigned int getNumDevices();

		////////////////////////////////////////////////////////////////////////////
		/*
		INTERNAL VARIABLES:

		If you need to declare any local variables or methods for your device,
		you may do it here bellow.
		*/
		////////////////////////////////////////////////////////////////////////////
		//magTrackerThread ourMagTracker;
		chai3d::cTransform pose;
		chai3d::cTransform poseCache;
		double scaleFactor;
		chai3d::cVector3d pos;
		chai3d::cVector3d centerPoint;
		chai3d::cVector3d scaledCenterPoint;
		chai3d::cVector3d scaledPos;


	protected:

		// the chai device has a 3dofdevice
		//c3DOFDevice* wearableDelta;
		//gripper* m_gripper;

		double gripperStartAngle = 0; // PI / 4;

		/* instead of connecting the device to the chai device, gripperChaiDevice passes 
		forces and torques back to the haptics thread, and the haptics thread sets the 
		gripper forces and torques to be used in the gripper object */

	};

	//------------------------------------------------------------------------------
}       // namespace chai3d
		//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
		//------------------------------------------------------------------------------

