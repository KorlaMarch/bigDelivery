#include "CVehicleVelCmd_TruckDrive.h"
#include <mrpt/utils/CStream.h>

using namespace mrpt::kinematics;
using namespace mrpt::utils;

CVehicleVelCmd_TruckDrive::CVehicleVelCmd_TruckDrive() :
	speed(.0),
	angle(.0)
{
}
CVehicleVelCmd_TruckDrive::~CVehicleVelCmd_TruckDrive()
{
}
size_t CVehicleVelCmd_TruckDrive::getVelCmdLength() const
{
	return 2;
}

std::string CVehicleVelCmd_TruckDrive::getVelCmdDescription(const int index) const
{
	switch (index)
	{
	case 0: return "speed"; break;
	case 1: return "angle"; break;
	default:
		THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

double CVehicleVelCmd_TruckDrive::getVelCmdElement(const int index) const
{
	switch (index)
	{
	case 0: return speed; break;
	case 1: return angle; break;
	default:
		THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

void CVehicleVelCmd_TruckDrive::setVelCmdElement(const int index, const double val)
{
	switch (index)
	{
	case 0: speed = val; break;
	case 1: angle = val; break;
	default:
		THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

bool CVehicleVelCmd_TruckDrive::isStopCmd() const
{
	return speed == .0;
}

void CVehicleVelCmd_TruckDrive::setToStop()
{
	speed = .0;
}

void CVehicleVelCmd_TruckDrive::cmdVel_scale(double vel_scale)
{
	speed *= vel_scale;
	angle *= vel_scale;
}

double CVehicleVelCmd_TruckDrive::cmdVel_limits(const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd, const double beta, const TVelCmdParams &params)
{
	ASSERT_(params.robotMax_V_mps>0);
	const CVehicleVelCmd_TruckDrive *prevcmd = dynamic_cast<const CVehicleVelCmd_TruckDrive*>(&prev_vel_cmd);
	ASSERTMSG_(prevcmd, "Expected prevcmd of type `CVehicleVelCmd_TruckDrive`");

	double speed_scale = 1.0;

	speed = beta*speed + (1 - beta)*prevcmd->speed;   // blend new v value

	// Speed limit
	if (std::abs(speed) > params.robotMax_V_mps) {
		// Scale:
		const double F = std::abs(params.robotMax_V_mps / speed);
		speed *= F;
		speed_scale *= F;
	}

	return speed_scale;
}