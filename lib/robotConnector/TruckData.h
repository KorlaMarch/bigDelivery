
#ifndef _TRUCKDATA_H_
#define _TRUCKDATA_H_

#include <iostream>
#include <fstream>

using namespace std;

class TruckData
{
public :
	
	int encoderLeft; // Encoder value at the left wheel
	int encoderRight; // Encoder value at the right wheel
	int encoderPivot; // Encoder value at the pivot point
	double driveAngle; // The angle between drivetrain and payload

	enum DriveControlMode
	{
		velocityRL = 0,
		speedAndAngle
	};
	DriveControlMode driveMode;

	// data for drive control (velocityRL mode)
	int requestVelLeft;
	int requestVelRight;

	// data for drive control (speedAndAngle mode)
	double requestSpeed;
	double requestAngle;

	// constants (robot body)
	double encoderStepPerMeter = 151440.4;
	double encoderStepPreRad = 320.22;
	double wheelDistance = 0.4235;
	double speed_coeff = 1666; // the speed sent to the board will be calculated by [ = speed_coeff * speed (in m/s) ]
	int speed_limit = 1000; // limitation of a speed that will be send to the wheels

	// constants (PID controller)
	double pid_p = 2000.0;
	double pid_d = 100.0;
	double pid_i = 20.0;
	double pid_stoppingThreshold = 0.03;
	double pid_frontbackThreshold = 0.50;

	TruckData()
	{

	}

	void Copy(TruckData &src)
	{
		encoderLeft = src.encoderLeft;
		encoderRight = src.encoderRight;
		encoderPivot = src.encoderPivot;
		driveAngle = src.driveAngle;

		driveMode = src.driveMode;

		requestVelLeft = src.requestVelLeft;
		requestVelRight = src.requestVelRight;

		requestSpeed = src.requestSpeed;
		requestAngle = src.requestAngle;

		encoderStepPerMeter = src.encoderStepPerMeter;
		encoderStepPreRad = src.encoderStepPreRad;
		wheelDistance = src.wheelDistance;
		speed_coeff = src.speed_coeff;
		speed_limit = src.speed_limit;
	}

	friend istream& operator >> (istream &is, TruckData &m)
	{
		is >> m.encoderLeft;
		is >> m.encoderRight;
		is >> m.encoderPivot;
		is >> m.driveAngle;

		// is >> m.driveMode;

		is >> m.requestVelLeft;
		is >> m.requestVelRight;

		is >> m.requestSpeed;
		is >> m.requestAngle;

		is >> m.encoderStepPerMeter;
		is >> m.encoderStepPreRad;
		is >> m.wheelDistance;
		is >> m.speed_coeff;
		is >> m.speed_limit;

		return is;
	}

	friend ostream& operator << (ostream &os, TruckData &m)
	{
		os << m.encoderLeft << " ";
		os << m.encoderRight << " ";
		os << m.encoderPivot << " ";
		os << m.driveAngle << " ";

		// os << m.driveMode << " ";

		os << m.requestVelLeft << " ";
		os << m.requestVelRight << " ";

		os << m.requestSpeed << " ";
		os << m.requestAngle << " ";

		os << m.encoderStepPerMeter << " ";
		os << m.encoderStepPreRad << " ";
		os << m.wheelDistance << " ";
		os << m.speed_coeff << " ";
		os << m.speed_limit << " ";

		return os;
	}
};
#endif // _TRUCKDATA_H_
