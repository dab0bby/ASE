/**
 * \file     Astra.cpp
 *
 * \brief    Implementation of the Astra class.
 *
 * \author   Gennadi Eirich (genna.eirich@gmail.com)
 * \date     02.01.2017
 *
 * \version  0.1.0
 *
 * \note     Copyright (c) 2017, HsKA
 */


#include "Astra.hpp"

#include "boost/make_shared.hpp"


namespace ASC
{
    namespace Device
    {
        Astra::Astra()
        {
            // Initialize astra cameras
            astra::initialize();

            _streamSet = astra::StreamSet();
            _streamReader = _streamSet.create_reader();
            _depthStream = new astra::DepthStream(_streamReader.stream<astra::DepthStream>());
        }


        void Astra::startCapture() const
        {
            // Start dept stream
            _depthStream->start();
        }


        void Astra::stopCapture() const
        {
            // Stop depthstream
            _depthStream->stop();
        }


        cv::Mat Astra::getDepthFrame(const bool mirror)
        {
            auto frame = _getFrame(); // Frame from camera

            if (frame.is_valid())
            {
                return Astra::depthFrameToMat(frame, mirror);
            }

            return cv::Mat();
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr Astra::getPointCloud(const cv::Mat& frame, const int stepsize) const
        {
            auto coordinateMapper = _depthStream->coordinateMapper();

            if (!frame.empty())
            {
                auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                auto h = frame.rows / stepsize;
                auto w = frame.cols / stepsize;

                cloud->height = 1;
                cloud->width = w * h;
                cloud->resize(w * h);

                for (auto y = 0; y < h; y++)
                {
                    for (auto x = 0; x < w; x++)
                    {
                        if (frame.at<short>(y * stepsize, x * stepsize) != 0)
                        {
                            coordinateMapper.convert_depth_to_world(
                                x * stepsize, y * stepsize, frame.at<short>(y * stepsize, x * stepsize),
                                &(cloud->at(x + y * w).x),
                                &(cloud->at(x + y * w).y),
                                &(cloud->at(x + y * w).z));
                        }
                        else
                        {
                            cloud->at(x + y * w).x = std::numeric_limits<float>::quiet_NaN();
                            cloud->at(x + y * w).y = std::numeric_limits<float>::quiet_NaN();
                            cloud->at(x + y * w).z = std::numeric_limits<float>::quiet_NaN();
                        }
                    }
                }

                cloud->is_dense = false;

                return cloud;
            }

            return boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        }


        // Gets most recent frame from cam
        astra::DepthFrame Astra::_getFrame()
        {
            auto frame = _streamReader.get_latest_frame();
            return frame.get<astra::DepthFrame>();  // Get DepthFrame from capturen frame
        }


        cv::Mat Astra::depthFrameToMat(const astra::DepthFrame& depthFrame, const bool mirror)
        {
            const auto h = depthFrame.height();
            const auto w = depthFrame.width();

            cv::Mat image(h, w, CV_16UC1);
            depthFrame.copy_to(reinterpret_cast<short*>(image.data));
            if (mirror)
                cv::flip(image, image, 1);
            return image;
        }


        void Astra::autoScale(cv::Mat& input, const int opencvtype, double minimum, double maximum)
        {
            double minval, maxval;
            cv::minMaxIdx(input, &minval, &maxval);

            if (minimum == -1) minimum = minval;
            if (maximum == -1) maximum = maxval;

            auto typemax = 256.0;
            if (opencvtype == CV_8UC1)
                typemax = 256.0;
            else if (opencvtype == CV_16U)
                typemax = 65536.0;

            double a = typemax / (maximum - minimum);
            double b = -a * minimum;

            input.convertTo(input, opencvtype, a, b);
        }


        Astra::~Astra()
        {
            // stop capture
            stopCapture();

            // Delete depthstream
            delete _depthStream;

            // Close astra cameras
            astra::terminate();
        }
    };
};
