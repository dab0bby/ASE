#include "Analyzer.h"
#include <pcl/io/vtk_lib_io.h>

using namespace ASE;
using namespace std;
using namespace pcl;

void main()
{
    ASE::AnalyzerConfig config;
    config.enableViewer = true;
    config.forceUserFeedback = true;

    PolygonMesh mesh;
    io::loadPolygonFileSTL("../sample data/reference.stl", mesh);
    
    PointCloud<PointXYZ> object, scene;
    io::loadPCDFile("../sample data/object.pcd", object);
    io::loadPCDFile("../sample data/scene.pcd", scene);
    
    Analyzer analyzer(config);
    auto result = analyzer.run(scene, object, mesh);
    
     
}