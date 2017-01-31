#pragma once
#include <vector>

namespace ASE 
{
    class AnalyzerConfig
    {
    public:
        AnalyzerConfig()
        {
        }

        bool enableViewer = false;
        bool forceUserFeedback = false;

        struct
        {
            int referenceCloudSamplingSize = 200000;
            double referenceCloudGridFilterSize = 0.0025;

            double keypointsLeafSize = 0.1;

            int icpMaxIterations = 200;
            int icpStep = 1;
            double icpEpsilon = 5e-8;
        } preprocess;

        struct
        {
            double subtractionTolerance = 0.02;
            int noiseFilterK = 100;
            double noiseStdDevMul = 0.5;
            double clusterTolerance = 0.02;
            int clusterMinSize = 1;
            int clusterMaxSize = 100000;

        } objectExtraction;

        struct
        {
            bool prealignToConvexHull = true;
            bool postalignByScale = false;
            int icpMaxIterations = 200;
            int icpStep = 1;
            double icpEpsilon = 1e-12;
            double icpgEpsilon = std::numeric_limits<double>::quiet_NaN();
        } objectAlignment;

        struct
        {
            int objectNormalEstimationK = 50;
            double objectNormalEstimationRadius = 0.08;
            double objectTriangulationKMultiplier = 3.5;
            double objectTriangulationSearchRadius = 0.05;

        } analysis;

        struct
        {
            float backgroundColor[3] = { 0.25f, 0.1f, 0.3f };
            float textColor[3] = { 0, 0.5f, 0.7f };
            int fontSize = 12;
            double pointSize = 1;
            double keyPointSize = 5;
            double cloudOpacity = 1;

            std::vector<std::vector<int>> colors =
            {
                { 197,0,0 },{ 57,187,0 },{ 186,187,0 },{ 135,134,255 },{ 255,0,0 },{ 208,0,192 },{ 65,197,198 },
                { 94,0,0 },{ 22,87,0 },{ 83,83,0 },{ 29,15,255 },{ 124,0,114 },{ 34,114,114 }
            };

        } visualizer;
    };
}