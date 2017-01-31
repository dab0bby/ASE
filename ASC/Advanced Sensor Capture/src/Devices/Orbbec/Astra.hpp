/**
 * \file     Astra.hpp
 *
 * \brief    This class provides the interface for the Orbbec astra device.
 *
 * \author   Gennadi Eirich (genna.eirich@gmail.com)
 * \date     02.01.2017
 *
 * \version  0.1.0
 *
 * \note     Copyright (c) 2017, HsKA
 */


#pragma once


#include "astra/astra.hpp"
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


namespace ASC
{
    namespace Device
    {
        class Astra
        {
            public:
                Astra();
                ~Astra();

                /**
                 * \brief   Starts the capturing for this device.
                 */
                void startCapture() const;

                /**
                 * \brief   Stops capture for this device.
                 */
                void stopCapture() const;

                /**
                 * \brief   Returns the most current depthFrame
                 */
                cv::Mat getDepthFrame(const bool mirror = true);

                /**
                 * \brief   Returns the most current point cloud.
                 */
                pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(const cv::Mat& frame, const int stepsize = 1) const;

                /**
                 * \brief   Converts the astra::DepthFrame to an 16UC1 cv::Mat
                 */
                static cv::Mat depthFrameToMat(const astra::DepthFrame& depthFrame, const bool mirror = true);

                /**
                 * \brief   Scales a given Grayscale frame to given min and max values
                 */
                static void autoScale(cv::Mat& input, const int opencvtype, double minimum, double maximum);

            private:
                astra::StreamSet    _streamSet;
                astra::StreamReader _streamReader;
                astra::DepthStream* _depthStream{ nullptr };

                astra::DepthFrame _getFrame();

        };
    }
}
