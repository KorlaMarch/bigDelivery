#include "RobotConnector.h"

#define _USE_MATH_DEFINES
#include <math.h>

RobotConnector::RobotConnector()
	: ganglia(2), pivot_buff_pt(0), isConnected(false)
{
}

bool RobotConnector::Connect(const char *gangliaPort, const char *pivotPort)
{
	// Coonect to the motor driver
	// Connent to the ganglia driver board
	ganglia.init(string(gangliaPort), 2000000);
	ganglia.setId({ 43, 44 });
	ganglia.setTimeout(0, 500000, 0, 100000);
	ganglia.setReadEncoderAbs({ false, false});
	ganglia.setMotorBLDC({ true, true });
	ganglia.setMotorEnable({ true, true });
	ganglia.setMotorControlMode({ 1, 1 });

	if (!ganglia.connect())
	{
		std::cerr << "[RobotConnector] Connecting to Ganglia Error" << std::endl;
		return false;
	}

	if (!ganglia.setProfile())
	{
		std::cerr << "[RobotConnector] Setting Profile Error" << std::endl;
		return false;
	}

	if (!ganglia.startAllMotor())
	{
		std::cerr << "[RobotConnector] Starting Motors Error" << std::endl;
		return false;
	}

	// Connect to the pivot encoder
	TCHAR portT[MAX_PATH];

	const int portlen = strlen(pivotPort);
	for (int i = 0; i <= portlen; i++) {
		portT[i] = pivotPort[i];
	}

	if (!pivotSerial.Open(portT, 1200))
	{
		std::cerr << "[RobotConnector] Connecting to Pivot Encoder Error" << std::endl;
		return false;
	}

	pivotSerial.Set_DTR_State(false);

	isConnected = true;

	return true;
}

void RobotConnector::Disconnect()
{
	isConnected = false;

	ganglia.setAllVelocity({ 0, 0 });
	ganglia.stopAllMotor();
	ganglia.disconnect();

	pivotSerial.Close();
}

bool RobotConnector::ReadData(TruckData &data)
{
	data.Copy(_data);
	return true;
}

bool RobotConnector::DriveDirect(double velLeft, double velRight)
{
	_data.driveMode = TruckData::DriveControlMode::velocityRL;
	_data.requestVelLeft = (int)   (velLeft * _data.speed_coeff);
	_data.requestVelRight = (int) -(velRight * _data.speed_coeff);

	int maxvel = max(abs(_data.requestVelLeft), abs(_data.requestVelRight));
	if (maxvel > _data.speed_limit) {
		_data.requestVelLeft  = _data.requestVelLeft  * _data.speed_limit / maxvel;
		_data.requestVelRight = _data.requestVelRight * _data.speed_limit / maxvel;
		cout << "[RobotConnector] Warning: velocity excesses the limit, scale down." << endl;
	}

	return ganglia.setAllVelocity({ _data.requestVelLeft, _data.requestVelRight });
}

bool RobotConnector::DrivePID(double speed, double angle)
{
	// reset PID when switching mode
	if (_data.driveMode != TruckData::DriveControlMode::speedAndAngle)
	{
		_data.driveMode = TruckData::DriveControlMode::speedAndAngle;
		lastE = 0;
		lastI = 0;
		lastUpdate.Tic();
	}

	// limit angle to a safe range
	if (angle > M_PI_2)
	{
		angle = M_PI_2;
	}
	else if (angle < -M_PI_2) {
		angle = -M_PI_2;
	}

	_data.requestSpeed = speed;
	_data.requestAngle = angle;

	return true;
}

bool RobotConnector::Update()
{
	if (!RequestData())
	{
		// Stop all motor, just to be safe
		ganglia.setAllVelocity({ 0, 0 });
		return false;
	}
	// only update velocity if on PID (speedAndAngle mode)
	if ( _data.driveMode == TruckData::DriveControlMode::speedAndAngle)
	{
		// convert encoder value to the angle (rad)
		double current_angle = _data.encoderPivot / _data.encoderStepPreRad;
		// find error
		double angular_error = _data.requestAngle - current_angle;
		double time_diff = lastUpdate.Tac();

		int vel_left  = (int)  (_data.requestSpeed * _data.speed_coeff);
		int vel_right = (int) -(_data.requestSpeed * _data.speed_coeff);

		// if an error is less than threshold, skip the PID
		if (fabs(angular_error) > _data.pid_stoppingThreshold)
		{
			// if the error is too big, do only rotation
			if (fabs(angular_error) > _data.pid_frontbackThreshold)
			{
				vel_left = 0;
				vel_right = 0;
			}

			// calculate p, d, and i term
			double p_term = angular_error * _data.pid_p;
			double d_term = (angular_error - lastE) / time_diff * _data.pid_d;
			double i_term = lastI * _data.pid_i;

			vel_left  += (int)-(p_term + d_term + i_term);
			vel_right += (int)-(p_term + d_term + i_term);

			lastI = (angular_error + lastE) / 2.0 * time_diff;

			// printf("Set : %d %d (e %.3lf %.3lf %.3lf)\n", vel_left, vel_right, angular_error, _data.requestAngle, current_angle);
		}
		else {
			lastI = 0;
		}

		int maxvel = max(abs(vel_left), abs(vel_right));
		if (maxvel > _data.speed_limit) {
			vel_left = vel_left * _data.speed_limit / maxvel;
			vel_right = vel_right * _data.speed_limit / maxvel;
			cout << "[RobotConnector] Warning: velocity excesses the limit, scale down." << endl;
		}

		lastE = angular_error;
		lastUpdate.Tic();
		if (ganglia.setAllVelocity({ vel_left , vel_right }))
		{
			return true;
		}
		else {
			cerr << "[RobotConnector] Set velocity failed" << endl;
			return false;
		}
	}

	return true;
}

bool RobotConnector::RequestData()
{
	if( !pivotSerial.IsOpen() ) return false;

	// Use DTR to prevent buffer overflow
	pivotSerial.Set_DTR_State(true);
	int length = pivotSerial.Read(_buff, 16368, 2);
	pivotSerial.Set_DTR_State(false);

	if (length == -1)
	{
		cout << "Pivot Encoder Error : Serial Read " << endl;
		return false;
	}
	//cout << "buffer size " << length << endl;
	
	int i;
	bool isErr = false;

	// find the first newline (\n)
	for (i = 0; i < length && _buff[i] != '\n'; i++);

	if (i >= length)
	{
		cout << "Pivot Encoder: no data received" << endl;
		return false;
	}

	// get the first complete encoder value
	pivot_buff_pt = 0;
	for (i++; i < length; i++)
	{
		if (_buff[i] == '\n')
		{
			pivot_buff[pivot_buff_pt] = '\0';
			_data.encoderPivot = atoi(pivot_buff);
			pivot_buff_pt = 0;
			break;
		}
		else if ( (_buff[i] >= '0' && _buff[i] <= '9') || _buff[i] == '-' ) {
			if (pivot_buff_pt >= 32)
			{
				// overflow
				cout << "Warning : Buffer overflow at pivot encoder" << endl;
				cout << "Dumping Data...\n";
				for (int j = 0; j < 32; j++)
				{
					cout << pivot_buff[j];
				}
				cout << endl;
				isErr = true;
				break;
			}
			pivot_buff[pivot_buff_pt++] = _buff[i];
		}
		else {
			cout << "Invalid Charactor Received from Pivot Encoder (" << _buff[i] << ")" << endl;
		}
	}
	if (isErr)
	{
		_buff[length] = '\0';
		cout << "Get: \n" << _buff << "\nend" << endl;
		return false;
	}

	// Get Encoder data from ganglia board
	ganglia::GangliaStateArray gangliaSA;
	ganglia.getAllGangliaState(gangliaSA);
	_data.encoderLeft = gangliaSA.data[0].encoder;
	_data.encoderRight = -gangliaSA.data[1].encoder;

	return true;
}

bool RobotConnector::getIsConnected()
{
	return isConnected;
}