#include "motorcontrol.h"
// --------------- TO DO ------------- SET ALL THESE VALUES -------------
#define PCI_BOARD 15           // only 1 826 board (number follows dip-switch code from manual section 2.2)
#define MODE_ENC  0x00000070  // for quadrature-encoded device using x4 clock multiplier
#define MODE_SNP  0x00000010  // automatically trigger counter snapshot on index pulse
#define QUAD_ERR  0x00000100  // counter snapshot triggered because of quadrature error
#define IX_RISE   0x00000010  // counter snapshot triggered on index rising edge
#define MAXCOUNT  0xFFFFFFFF  // maximum number of counts for 32-bit counter channel
#define MTR_RUN   0           // flag for motors to be operating normally
#define MTR_SAFE  1           // flag for motors to be in "safe" mode
#define VOLTRANGE 2           // output voltage range (2 = -5 to 5V, 3 = -10 to 10V)
#define MAXSETPNT 0xFFFF      // maximum analog output level (0x0000 to 0xFFFF covers output voltage range)
#define CNTPERREV 500         // Maxon HEDL5540 resolution [cnts/rev]
#define K_TORQ    0.127       // Maxon RE65 torque constant [N*m/A]
#define I_MAX     1			  // maximum current rating of motors [A]
#define VMAX	1			// maximum safe voltage for motors
#define VMIN	-1			//minimum safe voltage for motors
#define V_TO_I	 .1			  // amplifier gain
#define VRANGE_LOW -5
#define VRANGE_HI 5
#define PI        3.141592
#define DEBUG     0

bool connectToS826()
{
    int fail  = S826_SystemOpen();
    if (fail < 0) {
        return (false);
    } else {
        return (true);
    }
}

void disconnectFromS826()
{
    S826_SystemClose();
}

bool initMotor(uint channel)
{
    // set output range and initialize to 0 V = 1/2 max setpoint
    int fail  = S826_DacRangeWrite(PCI_BOARD, channel, VOLTRANGE, MTR_RUN);
        fail += S826_DacDataWrite(PCI_BOARD, channel, MAXSETPNT/2, MTR_RUN);

    // check for errors
    if (fail < 0) {
        return (false);
    } else {
        return (true);
    }
}

bool initEncod(uint channel)
{
    // enable channel and set to quadrature-encoded mode
    int fail  = S826_CounterModeWrite(PCI_BOARD, channel, MODE_ENC);
        fail += S826_CounterStateWrite(PCI_BOARD, channel, 1);

    // set counts for channel to center of range
        fail += setCounts(channel, (MAXCOUNT-1)/2);

    // set up automatic snapshots upon index pulse
        fail += S826_CounterSnapshotConfigWrite(PCI_BOARD, channel, MODE_SNP, 0);

    // check for errors
    if (fail < 0) {
        return (false);
    } else {
        return (true);
    }
}

bool checkEncod(uint channel)
{
    // probe snapshot buffer
    static uint snap_counts;
    static uint snap_reason;
    int fail = S826_CounterSnapshotRead(PCI_BOARD, channel, &snap_counts, NULL, &snap_reason, 0);

    // if snapshot available, check reason
    if (fail < 0) {
        return (true);  // no snapshot available, so no errors
    } else {
        if (snap_reason & QUAD_ERR) {
            if (DEBUG) {
                cout << "QUADRATURE ERROR" << endl;
				cout << "Ch  #" << channel << " = " << snap_counts << " cnts";
				cout << " " << endl ;
            }
            return (false);
        }
        else if (snap_reason & IX_RISE) {
            if (DEBUG) {
				std::cout << "INDEX LINE PULSE" << endl;
				std::cout << "Ch  #" << channel << " = " << snap_counts << " cnts";
				cout << " " << endl;
            }
            return (true);
        } else {
            return (true);
        }
    }
}

void setVolts(uint channel, double V)
{
    // check commanded voltage against dafe range
	if (channel != 4) {
		if (V > VMAX)  V = VMAX;
		if (V < VMIN)  V = VMIN;
	}

    // map voltage range to [0x0000,0xFFFF]
    uint setpnt = (V-VRANGE_LOW)/(VRANGE_HI-VRANGE_LOW) * MAXSETPNT;
    S826_DacDataWrite(PCI_BOARD, channel, setpnt, MTR_RUN);
}



void setTorque(uint channel, double T)
{
    // convert desired torque to (approximate) command voltage
    double I = T / K_TORQ;						// ------ TO DO -------
    if (fabs(I) > I_MAX) I = I_MAX;
    double V = I / V_TO_I;
    setVolts(channel, V);

    // print commanded torque and voltage for debugging
    if (DEBUG) {
		cout << "Ch  #" << channel << " = " << T << " N" << endl;
		cout << "Ch  #" << channel << " = " << V << " V" << endl;
    }
}

int setCounts(uint channel, uint counts)
{
    // write counts to preload register then copy preload to counter core
    int fail  = S826_CounterPreloadWrite(PCI_BOARD, channel, 0, counts);
        fail += S826_CounterPreload(PCI_BOARD, channel, 1, 0);

    return(fail);
}

int getCounts(uint channel)
{
    // manually read encoder
    static uint counts;
    S826_CounterRead(PCI_BOARD, channel, &counts);

    // center about middle of range
    if (counts >= (MAXCOUNT-1)/2) {
        return (int)counts - (MAXCOUNT-1)/2;
    } else {
        return -(int)((MAXCOUNT-1)/2 - (int)counts);
    }
}

double getAngle(uint channel, int zero)
{
    // read raw value from encoder and subtract offset
    int countsOff = getCounts(channel) - zero;

    // print counts for debugging
	if (DEBUG) cout << "Ch  #" << channel << " = " << countsOff << " cnts" << endl;

    // convert to radians, accounting for quadrature
    double angle = countsOff * (2.0*PI)/(CNTPERREV*4.0);

    // print motor angle for debugging
	if (DEBUG) cout << "Mtr #" << channel << " = " << angle*(180 / PI) << " deg" << endl;

    return angle;
}

double angleDiff(double a_thA, double a_thB)
{
	// determine shortest distance between A and B
	double diff1 = fabs(a_thA - a_thB);
	double diff2 = 2 * PI - diff1;
	double diff = fmin(diff1, diff2);

	// determine direction for moving from B to A along shortest path
	if (abs(fmod(a_thB + diff, 2 * PI) - a_thA) < THRESH)  return  1.0*diff;
	else                                                 return -1.0*diff;
}