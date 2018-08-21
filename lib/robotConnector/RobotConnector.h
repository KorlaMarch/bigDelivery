
#ifndef _ROBOTCONNECTOR_H_
#define _ROBOTCONNECTOR_H_

#include "SerialPort.h"
#include "TruckData.h"

#include "ganglia_driver_lib/ganglia_multi_driver.h"
#include "ganglia_driver_lib/structs.h"


#include <mrpt/utils/CTicTac.h>

class RobotConnector
{
public :
	RobotConnector();

	bool Connect(const char *gangliaPort = "COM9", const char *pivotPort = "\\\\.\\COM11");
	void Disconnect();
	bool ReadData(TruckData &data);
	bool DriveDirect(double velLeft, double velRight); // drive wheels with controlled speed (m/s) 
	bool DrivePID(double speed, double angle); // drive wheels with speed and angle (m/s, rad)
	bool Update();
	bool getIsConnected();

private:
	SerialPort pivotSerial;
	char _buff[16368];
	char pivot_buff[33];
	int  pivot_buff_pt;
	bool isConnected;

	double lastE = 0.0;
	double lastI = 0.0;
	mrpt::utils::CTicTac lastUpdate;

	ganglia::GangliaMultiDriver ganglia;
	TruckData _data;

	bool RequestData();
};
#endif // _ROBOTCONNECTOR_H_