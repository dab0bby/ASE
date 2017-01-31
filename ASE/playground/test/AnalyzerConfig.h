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
            double icpEpsilon = 4e-7;
        } preprocess;

        struct
        {
            double subtractionTolerance = 0.0025;
            int noiseFilterK = 100;
            double noiseStdDevMul = 0.5;
            double clusterTolerance = 0.02;
            int clusterMinSize = 1;
            int clusterMaxSize = 100000;

        } objectExtraction;

        struct
        {
            bool prealignToConvexHull = true;
            int icpMaxIterations = 200;
            int icpStep = 1;
            double icpEpsilon = 1e-12;
            double icpgEpsilon = 1e-30;
        } objectAlignment;

        struct
        {
            int backgroundColor[3] = { 63, 25, 26 };
            double pointSize = 1;
            double keyPointSize = 5;
            double cloudOpacity = 0.9;

            std::vector<int[3]> colors =
            {
                { 197,0,0 },{ 57,187,0 },{ 186,187,0 },{ 255,0,0 },{ 135,134,255 },{ 208,0,192 },{ 65,197,198 },
                { 94,0,0 },{ 22,87,0 },{ 83,83,0 },{ 29,15,255 },{ 124,0,114 },{ 34,114,114 }
            };

        } visualizer;

    private:
    };
}