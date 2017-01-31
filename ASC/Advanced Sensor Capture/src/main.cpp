/**
 * \file     main.cpp
 *
 * \brief    ASC Application entrance.
 *
 * \author   Gennadi Eirich (genna.eirich@gmail.com)
 * \date     03.01.2017
 *
 * \version  0.1.0
 *
 * \note     Copyright (c) 2017, HsKA
 */


#include <iostream>

#include "Devices/Orbbec/Astra.hpp"
#include "FileFactory.hpp"

#include "opencv2/opencv.hpp"
#include "boost/make_shared.hpp"
#include "pcl/visualization/pcl_visualizer.h"


int main(int argc, char* argv[])
{
    std::cout << "--- Advanced Sensor Capture ---\n" << std::endl;

    std::string path{};

    if (argc == 2)
        path = argv[1];

    cv::Mat frame;
    auto c = 0;
    auto iteration = 0;

    // Create cam
    auto cam = new ASC::Device::Astra();

    // Start capture
    cam->startCapture();

    // Setup pcl visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud");
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cam->getPointCloud(frame));
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    viewer->addCoordinateSystem(1.0);

    // Create FileFactory
    auto ff = ASE::FileFactory(path);

    while (c != 27 && !viewer->wasStopped())
    {
        // Retrieve frame
        frame = cam->getDepthFrame();

        if (c == 's')
        {
            std::cout << "Creating Scene PointCloud ..." << std::endl;

            // Get pointcloud
            auto cloud = cam->getPointCloud(frame);
            viewer->updatePointCloud(cloud);

            // Add to FileFactory
            ff.addSceneCloud(cloud);
        }
        else if (c == 'o')
        {
            std::cout << "Creating Scene with Object PointCloud ..." << std::endl;

            // Get pointcloud
            auto cloud = cam->getPointCloud(frame);
            viewer->updatePointCloud(cloud);

            // Add to FileFactory
            ff.addObjectCloud(cloud);
        }

        // Show depthframe
        {
            // Scale depthFrame
            ASC::Device::Astra::autoScale(frame, CV_16UC1, 0.0, 5000.0);

            if (!frame.empty())
                cv::imshow("DepthStream", frame);
        }

        viewer->spinOnce(30);
        c = cv::waitKey(30);
    }

    // Close windows
    cv::destroyAllWindows();
    viewer->close();

    // Save clouds to file
    std::cout << "\nSaving ..." << std::endl;
    if (ff.save())
        std::cout << "Successfull\n" << std::endl;
    else
        std::cerr << "Failure\n" << std::endl;

    // Do some cleanup
    delete cam;
    viewer.reset();

#if defined (_WIN32)
    system("Pause");
#endif // _WIN32

    return EXIT_SUCCESS;
}
