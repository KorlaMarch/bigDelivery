#ifndef BOTMODEL_H
#define BOTMODEL_H

#include "TruckData.h"
#include "RobotConnector.h"

#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>

#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/link_pragmas.h>

#include <mrpt/nav.h>

#define CONFIG_FILENAME "deliverybot-config.ini"

using namespace mrpt;

class BotModel {
public: 
	// Common Variable
	bool						allThreadsMustExit = false;
	bool						isStart = false;
	utils::CConfigFile*			cfgFile;
	maps::CMultiMetricMap		metricMap;
	math::TPose2D				targetPose;

	// Data From Sensor Thread -------------------------
	hwdrivers::CGenericSensor::TListObservations global_list_obs;
	synch::CCriticalSection     cs_global_list_obs;

	// Data From Manual Control Thread -------------------------
	RobotConnector				robot;
	TruckData					robotData;
	synch::CCriticalSection     cs_robotData;
	bool						isManual = true;
	int							encoder_left = 0;
	int							encoder_right = 0;

	// Data From Action/SF Generator -------------------------
	system::TTimeStamp			ASFTime = 0;
	obs::CActionCollection		actions;
	obs::CSensoryFrame			sf;
	poses::CPose2D				odometry;
	math::TTwist2D				estVelocity;
	maps::CSimplePointsMap		latest_obstacles;
	synch::CCriticalSection     cs_ASF;
	synch::CCriticalSection     cs_obstacles;

	// Data From PF Localizer -------------------------
	poses::CPose2D				locationEstimation;
	math::CMatrixDouble33		locationCov;
	synch::CCriticalSection     cs_location;
	slam::CMonteCarloLocalization2D* pdf = 0;
	synch::CCriticalSection     cs_pdf;

	// Data From Reactive Navigation -------------------------
	nav::CReactiveNavigationSystem* react_nav;

	// Methods
	BotModel();
	~BotModel();
	void loadGridMap(string map_file);

	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
	}
};

#endif