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
	double speed = lin_vel;
	double angle;
	

	if (lin_vel == 0.0)
	{
		angle = 0;
	}
	else {
		double sin_angle = ang_vel * botData->robotData.frontBackWheelsDistance / lin_vel;
		if (sin_angle > 1.0 || sin_angle < -1.0)
		{
			cout << "Warning: Impossible movement" << endl;
			angle = 0;
			speed = 0;
		}
		else {
			angle = asin(sin_angle);
		}
	}
	
	if ( !botData->robot.DrivePID(speed, angle) )
	{
		cerr << "SetControl Fail" << endl;
		return false;
	}
	return true;
}