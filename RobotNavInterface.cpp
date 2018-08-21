#include "RobotNavInterface.h"

bool RobotNavInterface::getCurrentPoseAndSpeeds(TPose2D &curPose, TTwist2D &curVel, TTimeStamp &timestamp, TPose2D &curOdometry, string &frame_id)
{
	curPose = botData->locationEstimation;
	curVel = botData->estVelocity;
	timestamp = mrpt::system::now();
	curOdometry = botData->odometry;
	return true;
}

bool RobotNavInterface::changeSpeeds(const CVehicleVelCmd &vel_cmd)
{
	if (botData->isStart && !botData->isManual)
	{
		const CVehicleVelCmd_DiffDriven* cmd = dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
		ASSERTMSG_(cmd, "Wrong vehicle kinematic class, expected `CVehicleVelCmd_DiffDriven`");
		updateMovement(cmd->lin_vel, cmd->ang_vel);
	}
	return true;
}

bool RobotNavInterface::changeSpeedsNOP()
{
	return true;
}

bool RobotNavInterface::stop(bool isEmergencyStop)
{
	if (botData->isStart && !botData->isManual)
	{
		mrpt::kinematics::CVehicleVelCmd_DiffDriven cmd;
		cmd.setToStop();
		changeSpeeds(cmd);
	}
	return true;
}

mrpt::kinematics::CVehicleVelCmdPtr RobotNavInterface::getStopCmd()
{
	mrpt::kinematics::CVehicleVelCmdPtr cmd(new mrpt::kinematics::CVehicleVelCmd_DiffDriven());
	cmd->setToStop();
	return cmd;
}
mrpt::kinematics::CVehicleVelCmdPtr RobotNavInterface::getEmergencyStopCmd()
{
	return getStopCmd();
}

bool RobotNavInterface::senseObstacles(maps::CSimplePointsMap &obstacles, TTimeStamp &timestamp)
{

	{
		synch::CCriticalSectionLocker obstacles_lock(&botData->cs_obstacles);
		obstacles = botData->latest_obstacles;
	}
	
	timestamp = botData->ASFTime;
	return true;
}

bool RobotNavInterface::updateMovement(double lin_vel, double ang_vel)
{
	double left_vel  = lin_vel - ang_vel * wheel_radius;
	double right_vel = lin_vel + ang_vel * wheel_radius;

	// if the velocity exceed the maximum, turn it down
	const double maxVal = max(fabs(left_vel), fabs(right_vel));
	if (maxVal > 0.6/1000.0) {
		left_vel  /= maxVal;
		right_vel /= maxVal;
	}

	if ( !botData->robot.DriveDirect((int)(left_vel*1000), (int)(right_vel*1000)) )
	{
		cerr << "SetControl Fail" << endl;
		return false;
	}
	return true;
}