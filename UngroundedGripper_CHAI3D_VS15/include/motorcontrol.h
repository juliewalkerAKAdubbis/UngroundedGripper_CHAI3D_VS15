#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "826api.h"
#include <cmath>
#include <iostream>
#include <algorithm>

#define THRESH 0.001

// important values for motors
// 0: index distal
// 1: index proximal
// 2: thumb proximal
// 3: thumb distal
// 4: grip force motor


	const double gearRatio[5] = { 64.0, 64.0, 64.0, 64.0, 30.0 };
	const double encoderLinesPerRev[5] = { 200, 200, 200, 200, 4.0 }; // 12800  for micromo 50*4(*64:1), for pololu (30*)4
	const double k_torq[5] = { 0.12544, 0.12544, 0.12544, 0.12544, 0.035 };	//	[N-m/A]		--------- TO DO -------
	const double i_max[5] = { 0.1, 0.1, 0.1, 0.1, 1 };	
	const double amp_gain[5] = { 0.1, 0.1, 0.1, 0.1, 0.1 };


bool connectToS826();
void disconnectFromS826();
bool initMotor(uint channel);
bool initEncod(uint channel);
bool checkEncod(uint channel);
void setVolts(uint channel, double V);
void setTorque(uint channel, double T);
int setCounts(uint channel, uint counts);
int getCounts(uint channel);
double getAngle(uint channel);
double angleDiff(double a_thA, double a_thB);

using namespace std;

#endif // MOTORCONTROL_H
