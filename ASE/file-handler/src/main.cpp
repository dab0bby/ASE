/**
 * Main function to test the FileFactory and FileReader
 */


#define _SCL_SECURE_NO_WARNINGS 1


#include "FileFactory.hpp"
#include "FileReader.hpp"

#include <boost/smart_ptr/make_shared.hpp>
#include <iostream>


int main(int argc, char* argv[])
{
    // FileFactory
    {
        std::cout << "Testing FileFactory ..." << std::endl;

        // Some test clouds
        auto cloud0 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(300, 300, pcl::PointXYZ(0.5, 0.5, 0.5));
        auto cloud1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // Empty cloud

        auto ff00 = ASE::FileFactory();
        ff00.addSceneCloud(cloud0);
        ff00.addObjectCloud(cloud0);

        auto ff01 = ASE::FileFactory("..\\");
        ff01.addSceneCloud(cloud0);
        ff01.addObjectCloud(cloud1);

        auto ff02 = ASE::FileFactory();

        // Do some tests
        assert(ff00.save());     // Valid clouds
        assert(!ff01.save());    // A cloud is missing
        assert(!ff02.save());    // No clouds

        std::cout << "[ OK ]\n" << std::endl;
    }

    // FileReader
    {
        std::cout << "Testing FileReader ..." << std::endl;

        auto fr00 = ASE::FileReader();
        auto fr01 = ASE::FileReader();

        assert(fr00.getSceneCloud()->empty());
        assert(!fr00.loadSceneCloud("scene_cloud.pcd")->empty());
        assert(!fr00.getSceneCloud()->empty());
        assert(fr00.getObjectCloud()->empty());
        assert(fr01.getObjectCloud()->empty());
        assert(!fr01.loadObjectCloud("object_cloud.pcd")->empty());
        assert(!fr01.getObjectCloud()->empty());
        assert(fr01.getSceneCloud()->empty());

        assert(fr01.loadObjectCloud("fileThatDoesNotExist.pcd")->empty());  // Invalid file
        assert(fr01.getObjectCloud()->empty());                             // Should be empty now coz no valid file was loaded

        std::cout << "[ OK ]\n" << std::endl;
    }

#if defined (_WIN32)
    system("Pause");
#endif

    return EXIT_SUCCESS;
}
