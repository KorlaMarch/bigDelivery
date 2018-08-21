#include "BotModel.h"

using namespace mrpt::system;

BotModel::BotModel()
{

	const string CFG_FILENAME = string(CONFIG_FILENAME);
	ASSERT_FILE_EXISTS_(CFG_FILENAME)
	cfgFile = new mrpt::utils::CConfigFile(CFG_FILENAME);

	// Load Grid Map
	string mapFile = cfgFile->read_string("MapFiles", "mapFile", "", true);
	loadGridMap(mapFile);
}

BotModel::~BotModel()
{

}

void BotModel::loadGridMap(string map_file) {
	using namespace mrpt::utils;

	maps::TSetOfMetricMapInitializers	mapList; // Metric map options
	mapList.loadFromConfigFile(*this->cfgFile, "MetricMap");

	metricMap.setListOfMaps(&mapList);

	ASSERT_FILE_EXISTS_(map_file);

	// Detect file extension:
	// -----------------------------
	string mapExt = lowerCase(extractFileExtension(map_file, true)); // Ignore possible .gz extensions

	if (!mapExt.compare("simplemap"))
	{
		maps::CSimpleMap	simpleMap;
		// It's a ".simplemap":
		// -------------------------
		printf("Loading '.simplemap' file...");
		CFileGZInputStream(map_file) >> simpleMap;
		printf("Ok\n");

		ASSERT_(simpleMap.size()>0);

		// Build metric map:
		// ------------------------------
		cout << "Building metric map(s) from '.simplemap'...";
		metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
		cout << "Ok" << endl;
	}
	else if (!mapExt.compare("gridmap"))
	{
		// It's a ".gridmap":
		// -------------------------
		cout << "Loading gridmap from '.gridmap'...";
		ASSERT_(metricMap.m_gridMaps.size() == 1);
		CFileGZInputStream(map_file) >> (*metricMap.m_gridMaps[0]);
		cout << "Ok" << endl;
	}
	else
	{
		// Try loading the image:
		CImage img;
		if (!img.loadFromFile(map_file, 0 /* force grayscale */))
		{
			throw std::runtime_error(string("***ERROR***: Cannot load map from file: ") + map_file);
		}
		else
		{
			// We also need the size of each pixel:
			double cx;
			double cy;
			double cresolution;

			cx = cfgFile->read_double("MapFiles", "imgCenter_x", -1, false);
			cy = cfgFile->read_double("MapFiles", "imgCenter_y", -1, false);
			cresolution = cfgFile->read_double("MapFiles", "imgResolution", 0.04, false);

			ASSERT_(metricMap.m_gridMaps.size() == 1);
			if (!(*metricMap.m_gridMaps[0]).loadFromBitmap(img, cresolution, cx, cy))
			{
				throw std::runtime_error(string("***ERROR***: Cannot load the image into the gridmap."));
			}
		}

	}
}