#ifndef ROBOTNAVINTERFACE_H
#define ROBOTNAVINTERFACE_H

#include "BotModel.h"

#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/utils/CTicTac.h>

// CRobot2NavInterface implemented for whitebot

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::kinematics;

class RobotNavInterface : public CRobot2NavInterface
{
private:
	BotModel* botData;
public:
	RobotNavInterface(BotModel* _botData) : botData(_botData) {
	}

	bool getCurrentPoseAndSpeeds(TPose2D &curPose, TTwist2D &curVel, TTimeStamp &timestamp, TPose2D &curOdometry, string &frame_id) override;
	bool changeSpeeds(const CVehicleVelCmd &vel_cmd) override;
	bool changeSpeedsNOP() override;
	bool stop(bool isEmergencyStop) override;

	CVehicleVelCmdPtr getStopCmd() override;
	CVehicleVelCmdPtr getEmergencyStopCmd() override;

	bool senseObstacles(maps::CSimplePointsMap &obstacles, TTimeStamp &timestamp) override;
private: 
	// Update speed of the robot, velocities are in m/s and rad/s
	bool updateMovement(double lin_vel, double ang_vel);
};

#endif // ROBOTNAVINTERFACE_H