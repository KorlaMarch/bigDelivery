#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>


DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE(CVehicleVelCmd_TruckDrive, CVehicleVelCmd)

class CVehicleVelCmd_TruckDrive : public mrpt::kinematics::CVehicleVelCmd
{
	DEFINE_SERIALIZABLE(CVehicleVelCmd_TruckDrive)
public:
	double speed; //!< Speed of both wheels (m/s)
	double angle; //!< Angle between drive train and body (rad)

	CVehicleVelCmd_TruckDrive();
	virtual ~CVehicleVelCmd_TruckDrive();
	size_t getVelCmdLength() const MRPT_OVERRIDE;
	std::string getVelCmdDescription(const int index) const MRPT_OVERRIDE;
	double getVelCmdElement(const int index) const  MRPT_OVERRIDE;
	void setVelCmdElement(const int index, const double val) MRPT_OVERRIDE;
	bool isStopCmd() const MRPT_OVERRIDE;
	void setToStop() MRPT_OVERRIDE;

	void cmdVel_scale(double vel_scale) MRPT_OVERRIDE;

	double cmdVel_limits(const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd, const double beta, const TVelCmdParams &params)  MRPT_OVERRIDE;
};

DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE(CVehicleVelCmd_TruckDrive, CVehicleVelCmd)