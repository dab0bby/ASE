/**
 * \file     FileFactory.hpp
 *
 * \brief   This class provides the functionality to save a ASE file.
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


namespace ASE
{
    static const std::string SCENE_CLOUD_FILENAME  = "scene";
    static const std::string OBJECT_CLOUD_FILENAME = "object";

    class FileFactory
    {
        public:
            FileFactory() :
                _path(std::string())
            {
            };


            explicit FileFactory(const std::string& path) :
                _path(path)
            {
            };


            ~FileFactory()
            {
                // Save if clouds were not saved before
                if (!_saved)
                    save();

                // Clear instances
                _sceneCloud.reset();
                _objectCloud.reset();
            };


            /**
             * \brief   Add the scene cloud
             */
            void addSceneCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
            {
                // Reset previous cloud and add new one
                _sceneCloud.reset();
                _sceneCloud = cloud;
            };


            /**
             * \brief   Add the scene with the object cloud
             */
            void addObjectCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
            {
                // Remove previous cloud and add new one
                _objectCloud.reset();
                _objectCloud = cloud;
            };


            /**
             * \brief   Saves clouds to file.
             */
            bool save()
            {
                // Dont save if no clouds are given
                if (!_sceneCloud || !_objectCloud)
                    return false;

                // Save scene cloud
                if (!_sceneCloud->empty())
                    _save(_sceneCloud, SCENE_CLOUD_FILENAME);
                else
                    return false;

                // Save object cloud
                if (!_objectCloud->empty())
                    _save(_objectCloud, OBJECT_CLOUD_FILENAME);
                else
                    return false;

                _saved = true;
                return _saved;
            };

        private:
            const std::string _path;
            bool              _saved{ false };

            pcl::PointCloud<pcl::PointXYZ>::ConstPtr _sceneCloud;
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr _objectCloud;

            // ReSharper disable once CppMemberFunctionMayBeStatic
            void _save(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::string& filename) const
            {
                pcl::PCDWriter().write(_path + filename + ".pcd", *cloud);
            };

    };
}
