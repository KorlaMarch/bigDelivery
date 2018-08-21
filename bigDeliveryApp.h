/***************************************************************
 * Name:      bigDeliveryApp.h
 * Purpose:   Defines Application Class
 * Author:    Khemarat Boonyapaluk (korla.march@gmail.com)
 * Created:   2018-07-16
 * Copyright: Khemarat Boonyapaluk (korlamarch.com)
 * License:
 **************************************************************/

#ifndef BIGDELIVERYAPP_H
#define BIGDELIVERYAPP_H

#include "BotModel.h"

#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/slam/link_pragmas.h>

#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/random.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/utils.h>

#include <mrpt/nav.h>


#include <wx/app.h>

#include "RobotNavInterface.h"


class bigDeliveryApp : public wxApp
{
    public:
        virtual bool OnInit();
		static void ConnectorUpdateThread(BotModel* botData);
		static void ControlThread(BotModel* botData);
		static void SensorThread(BotModel* botData);
		static void ASFGeneratorThread(BotModel* botData); /* This is action/SF pair generator */
		static void PFLocalizerThread(BotModel* botData);
		static void ReactiveNavThread(BotModel* botData);
};

#endif // BIGDELIVERYAPP_H
