/**
 * \file     main.cpp
 *
 * \brief    ASE Application entrance.
 *
 * \author   Gennadi Eirich (genna.eirich@gmail.com)
 * \date     03.01.2017
 *
 * \version  0.1.0
 *
 * \note     Copyright (c) 2017, HsKA
 */


#define _SCL_SECURE_NO_WARNINGS 1


#include <iostream>
#include <fstream>

#include "../../file-handler/src/FileReader.hpp"
#include "Analyzer.h"
#include "AnalyzerConfig.h"

#include "Eigen/Geometry"
#include "pcl/console/parse.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/common/transforms.h"

using namespace std;

struct
{
    double scaleFactor = 0.001;
} cameraConfiguration;


// Analysis configuration
auto configuration = ASE::AnalyzerConfig();


/**
 * \brief Shows usage help on the console
 */
void showHelp()
{
    std::cout << "\nUsage: ASE.exe <scene_cloud_filename.pcd> <object_cloud_filename.pcd> <reference_object_filename.stl> <output_filename.txt> [-c <configfile.txt> -h -v -vf]" << std::endl;
    std::cout << "-c    Specifies a custom configuration file." << std::endl;
    std::cout << "-h    Shows this help." << std::endl;
    std::cout << "-v    Enables the viewer window. The whole process is visualized in this viewer." << std::endl;
    std::cout << "-vf   Same as -v. Furthermore, the process interrupts at several stages of the analysis and waits for the user to press a key. " << std::endl;
}


/**
 * \brief Loads configuration from file
 */
void loadConfiguration(const std::string& filename)
{
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line))
    {
        // Search for separater
        auto found = line.find(':');
        if (found != std::string::npos)
        {
            // Get key and value
            auto key = line.substr(0, found);
            auto value = line.substr(found + 1);

            // Get scale factor
            if (key == "scale_factor")
                cameraConfiguration.scaleFactor = std::stod(value);

            // Values for analysis
            else if (key == "user_feedback")
            {
                configuration.forceUserFeedback = value == "true";

                // Enable viewer if user wants to use feedback
                if (value == "true")
                    configuration.enableViewer = true;
            }
            else if (key == "reference_cloud_sampling_size")
                configuration.preprocess.referenceCloudSamplingSize = std::stoi(value);
            else if (key == "reference_cloud_grid_filter_size")
                configuration.preprocess.referenceCloudGridFilterSize = std::stod(value);
            else if (key == "keypoints_leaf_size")
                configuration.preprocess.keypointsLeafSize = std::stod(value);
            else if (key == "icp_max_iteration")
                configuration.preprocess.icpMaxIterations = std::stoi(value);
            else if (key == "icp_step")
                configuration.preprocess.icpStep = std::stoi(value);
            else if (key == "icp_epsilon")
                configuration.preprocess.icpEpsilon = std::stod(value);
            else if (key == "substraction_tolerance")
                configuration.objectExtraction.subtractionTolerance = std::stod(value);
            else if (key == "noise_filter_k")
                configuration.objectExtraction.noiseFilterK = std::stoi(value);
            else if (key == "noise_std_dev_mul")
                configuration.objectExtraction.noiseStdDevMul = std::stod(value);
            else if (key == "cluster_tolerance")
                configuration.objectExtraction.clusterTolerance = std::stod(value);
            else if (key == "cluster_min_size")
                configuration.objectExtraction.clusterMinSize = std::stoi(value);
            else if (key == "cluster_max_size")
                configuration.objectExtraction.clusterMaxSize = std::stoi(value);
            else if (key == "prealign_to_convex_hull")
                configuration.objectAlignment.prealignToConvexHull = value == "true";
            else if (key == "prealign_to_boundingbox")
                configuration.objectAlignment.prealignToBoundingBox = value == "true";            
            else if (key == "postalign_by_scale")
                configuration.objectAlignment.postalignByScale = value == "true";
            else if (key == "alignment_icp_max_iteration")
                configuration.objectAlignment.icpMaxIterations = std::stoi(value);
            else if (key == "alignment_icp_step")
                configuration.objectAlignment.icpStep = std::stoi(value);
            else if (key == "alignment_icp_epsilon")
                configuration.objectAlignment.icpEpsilon = std::stod(value);
            else if (key == "alignment_icpg_epsilon")
                configuration.objectAlignment.icpgEpsilon = std::stod(value);
            else if (key == "object_normal_estimation_k")
                configuration.analysis.objectNormalEstimationK = std::stoi(value);
            else if (key == "object_normal_estimation_radius")
                configuration.analysis.objectNormalEstimationRadius = std::stod(value);
            else if (key == "object_triangulation_multiplier")
                configuration.analysis.objectTriangulationKMultiplier = std::stod(value);
            else if (key == "object_triangulation_search_radius")
                configuration.analysis.objectTriangulationSearchRadius = std::stod(value);
        }
    }
}


/**
 * \brief   Prints the result
 */
void printResult(ostream& stream, const ASE::AnalysisResult& result, const std::string& objectFile, const std::string& modelFile, const std::string& sceneFile)
{
    const auto w = 7;
    stream << "Advanced Sensor Evaluation - Analysis Result" << endl;
    stream << endl;
    stream << "Background File: \"" << sceneFile << "\"" << endl;
    stream << "Foreground File: \"" << objectFile << "\"" << endl;
    stream << "Reference File:  \"" << modelFile << "\"" << endl;
    stream << endl;
    stream << "Object Area:      " << fixed << setw(w) << setprecision(3) << result.getObjectCloudArea() << " m²" << endl;
    stream << "Reference Area:   " << fixed << setw(w) << setprecision(3) << result.getReferenceCloudArea() << " m²" << endl;
    stream << "Area Coverage:    " << fixed << setw(w) << setprecision(3) << result.getAreaCoverage() * 100.0 << " %" << endl;
    stream << "Object Scale:     " << fixed << setw(w) << setprecision(3) << result.getObjectScale() << endl;
    stream << endl;
    stream << "Average Distance: " << fixed << setw(w) << setprecision(3) << result.getAverageDistance() * 1000 << " mm" << endl;
    stream << "Median Distance:  " << fixed << setw(w) << setprecision(3) << result.getMedianDistance() * 1000 << " mm" << endl;
    stream << "Minimum Distance: " << fixed << setw(w) << setprecision(3) << result.getQuantilDistance(0) * 1000 << " mm" << endl;
    stream << "Maximum Distance: " << fixed << setw(w) << setprecision(3) << result.getQuantilDistance(1) * 1000 << " mm" << endl;
    stream << "10% Quantile:     " << fixed << setw(w) << setprecision(3) << result.getQuantilDistance(0.1) * 1000 << " mm" << endl;
    stream << "25% Quantile:     " << fixed << setw(w) << setprecision(3) << result.getQuantilDistance(0.25) * 1000 << " mm" << endl;
    stream << "75% Quantile:     " << fixed << setw(w) << setprecision(3) << result.getQuantilDistance(0.75) * 1000 << " mm" << endl;
    stream << "90% Quantile:     " << fixed << setw(w) << setprecision(3) << result.getQuantilDistance(0.9) * 1000 << " mm" << endl;
    stream << endl;
    stream << "Computation Time: " << fixed << setw(w) << setprecision(3) << result.getComputationTime() << " s" << endl;
}


/**
 * \brief   Writes result to file
 */
void writeResult(const std::string& filename, const ASE::AnalysisResult& result, const std::string& objectFile, const std::string& modelFile, const std::string& sceneFile)
{
    fstream file(filename, std::ios_base::out);
    printResult(file, result, objectFile, modelFile, sceneFile);
    file.close();
}

int shutdown(int exitCode = EXIT_SUCCESS)
{
#if defined (_WIN32)
    cout << endl;
    system("Pause");
#endif

    return exitCode;
}

int main(int argc, char* argv[])
{
    std::cout << "Advanced Sensor Evaluation\n" << std::endl;

    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp();
        return shutdown();
    }

    if (argc < 5)
    {
        showHelp();
        return shutdown(EXIT_FAILURE);
    }

    const std::string sceneFile = argv[1];
    const std::string objectFile = argv[2];
    const std::string referenceFile = argv[3];
    const std::string outputFile = argv[4];

    std::cout << "Scene cloud:      " << sceneFile << std::endl;
    std::cout << "Object cloud:     " << objectFile << std::endl;
    std::cout << "Reference object: " << referenceFile << std::endl;
    std::cout << "Output data:      " << outputFile << std::endl;

    if (!ASE::FileReader::fileExists(sceneFile))
    {
        std::cerr << "Scene file does not exist." << std::endl;
        return shutdown(EXIT_FAILURE);
    }
    if (!ASE::FileReader::fileExists(objectFile))
    {
        std::cerr << "Object file does not exist." << std::endl;
        return shutdown(EXIT_FAILURE);
    }
    if (!ASE::FileReader::fileExists(referenceFile))
    {
        std::cerr << "Reference file does not exist." << std::endl;
        return shutdown(EXIT_FAILURE);
    }

    // Check if user wants to see the visualizer
    if (pcl::console::find_switch(argc, argv, "-v"))
        configuration.enableViewer = true;

    // Check if user wants to see the visualizer and a staged analysis with feedback
    if (pcl::console::find_switch(argc, argv, "-vf"))
    {
        configuration.enableViewer = true;
        configuration.forceUserFeedback = true;
    }

    {
        std::string configFileName;

        if (pcl::console::parse(argc, argv, "-c", configFileName) >= 0)
        {
            if (!ASE::FileReader::fileExists(configFileName))
            {
                std::cerr << "\nConfiguration file does not exist." << std::endl;
                return shutdown(EXIT_FAILURE);
            }

            std::cout << "Configuration:    " << configFileName << std::endl;

            // Load configuratio
            loadConfiguration(configFileName);
        }
    }
    std::cout << std::endl;

    // Read the data
    std::cout << "Loading ...";
    auto fr = ASE::FileReader();
    auto scene = fr.loadSceneCloud(sceneFile);    // Scene
    auto object = fr.loadSceneCloud(objectFile);   // Scene with object
    pcl::PolygonMesh reference;
    pcl::io::loadPolygonFileSTL(referenceFile, reference);
    std::cout << " Done" << std::endl;

    // Check if data is valid
    if (scene->empty() || object->empty())
    {
        std::cerr << "Cloud data not valid." << std::endl;
        return shutdown(EXIT_FAILURE);
    }

    // Scale the clouds
    {
        auto matrix = Eigen::Affine3f::Identity();
        matrix.scale(cameraConfiguration.scaleFactor);

        pcl::transformPointCloud(*scene, *scene, matrix);
        pcl::transformPointCloud(*object, *object, matrix);
    }

    // Run analysis
    auto analyzer = new ASE::Analyzer(configuration);
    auto result = analyzer->run(*scene, *object, reference);

    // Print result
    printResult(cout, result, objectFile, referenceFile, sceneFile);

    // Write result to file
    writeResult(outputFile, result, objectFile, referenceFile, sceneFile);

    // Cleanup
    delete analyzer;
    
    return shutdown();
}
