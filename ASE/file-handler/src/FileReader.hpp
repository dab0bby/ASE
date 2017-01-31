/**
 * \file     FileReader.hpp
 *
 * \brief   This class provides the functionality to read saved ASE file.
 *
 * \author   Gennadi Eirich (genna.eirich@gmail.com)
 * \date     02.01.2017
 *
 * \version  0.1.0
 *
 * \note     Copyright (c) 2017, HsKA
 */


#pragma once


#define _SCL_SECURE_NO_WARNINGS 1
#define _CRT_SECURE_NO_WARNINGS 1


#include <string>

#include "boost/make_shared.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"


namespace ASE
{
    class FileReader
    {
        public:
            FileReader() :
                _sceneCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
                _objectCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
            {};


            ~FileReader()
            {
                _sceneCloud.reset();
                _objectCloud.reset();
            };

            /**
             * \brief   Reads scene cloud from a given PCD file
             *          If file ist not valid an empty cloud is returned. getSceneCloud() will also return an empty cloud
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr loadSceneCloud(const std::string& filename)
            {
                if (fileExists(filename))
                    _sceneCloud = _read(filename);
                else
                    _sceneCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                return _sceneCloud;
            };


            /**
             * \brief   Returns loaded scene cloud.
             *          If not one was loaded an empty cloud is returned
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr getSceneCloud() const
            {
                return _sceneCloud;
            };


            /**
             * \brief   Reads scene with object cloud from a given PCD file
             *          If not valid file is given an empty cloud is returned. getOjectCloud() will also return an empty cloud
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr loadObjectCloud(const std::string& filename)
            {
                if (fileExists(filename))
                    _objectCloud = _read(filename);
                else
                    _objectCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                return _objectCloud;
            };


            /**
             * \brief   Returns loaded scene with object cloud.
             *          If not one was loaded an empty cloud is returned
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr getObjectCloud() const
            {
                return _objectCloud;
            };

            /**
             * \brief   Cheks if a given file exists
             */
            static bool fileExists(const std::string& filename)
            {
                if (FILE* file = fopen(filename.c_str(), "r"))
                {
                    fclose(file);
                    return true;
                }

                return false;
            }

        private:
            pcl::PointCloud<pcl::PointXYZ>::Ptr _sceneCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _objectCloud;

            /**
             * \brief   Reads the cloud from file
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr _read(const std::string& filename) const
            {
                auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                // Read from file
                pcl::PCDReader().read(filename, *cloud);

                return cloud;
            };

    };
}
