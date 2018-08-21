#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CJoystick.h>

#include "TruckData.h"
#include "RobotConnector.h"

int main(){
	try
	{
		RobotConnector robot;
		TruckData truck_data;

		float		x, y, z;
		mrpt::vector_bool	buttons;
		mrpt::hwdrivers::CJoystick	joy;

		// Connect to the Robot
		if (!robot.Connect("COM9", "\\\\.\\COM11"))
		{
			throw std::runtime_error(string("***Error***: Can't connect to the robot"));
		}

		robot.DriveDirect(0, 0);

		while (true)
		{
			robot.Update();
			if (!robot.ReadData(truck_data))
			{
				cout << "Robot : Read Data failed" << endl;
			}
			else if (joy.getJoystickPosition(0, x, y, z, buttons))
			{
				// Joystick Threshold
				x = fabs(x) < 0.1 ? 0 : x;
				y = fabs(y) < 0.1 ? 0 : y;

				cout << "Robot Request : " << -x * M_PI / 2.0 << endl;
				robot.DrivePID(-y / 2.0, -x * M_PI / 2.0);
			}
			else {
				cout << "Joystick reading error" << endl;
				// cout << "Robot Encoder :" << truck_data.encoderPivot << endl;
			}
			mrpt::system::sleep(105);
		}
	}
	catch (std::exception &e)
	{
		cerr << "Closing due to exception:\n" << e.what() << endl;
	}
	catch (...)
	{
		cerr << "Untyped exception! Closing." << endl;
	}
	mrpt::system::pause();
}