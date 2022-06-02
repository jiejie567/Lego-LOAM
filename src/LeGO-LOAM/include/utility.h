#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cv_bridge/cv_bridge.h>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;

// VLP-16
extern const int N_SCAN = 480;
extern const int Horizon_SCAN = 640;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

// extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 1.0/15.0;//15hz的相机，所以1/15
extern const int systemDelay = 0;
extern const int imuQueLength = 200;
extern const string imuTopic = "/imu/data";


extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472; // segmentTheta=1.0472<==>60度,在imageProjection中用于判断平面
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
// extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
// extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;
extern const int   surroundingKeyframeSearchNum = 50;

extern const float historyKeyframeSearchRadius = 5.0;
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 500.0;


struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;
class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    // int N_SCAN;
    // int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    //RGBD Sensor Configuration
    float fx;
    float fy;
    float ppx;
    float ppy;
    float depthFactor;
    bool imuDebug;
    float cameraHeight;//相机安装高度
    int groundScanInd;//开始识别地面点的行数

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float segmentDiff;//两个点聚类的深度差值的阈值
    float groundTh;//地面高度限制阈值
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lego_loam/pointCloudTopic", pointCloudTopic, "camera/aligned_depth_to_color/image_raw");
        nh.param<std::string>("lego_loam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lego_loam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lego_loam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("lego_loam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lego_loam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lego_loam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lego_loam/mapFrame", mapFrame, "map");

        nh.param<bool>("lego_loam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lego_loam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lego_loam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("lego_loam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("lego_loam/savePCD", savePCD, false);
        nh.param<std::string>("lego_loam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");


        // nh.param<int>("lego_loam/N_SCAN", N_SCAN, 480);
        // nh.param<int>("lego_loam/Horizon_SCAN", Horizon_SCAN, 640);
        nh.param<int>("lego_loam/downsampleRate", downsampleRate, 1.0);
        nh.param<float>("lego_loam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lego_loam/lidarMaxRange", lidarMaxRange, 200.0);
        nh.param<float>("lego_loam/fx", fx, 321.056);
        nh.param<float>("lego_loam/fy", fy, 243.269);
        nh.param<float>("lego_loam/ppx", ppx, 385.283);
        nh.param<float>("lego_loam/ppy", ppy, 385.283);
        nh.param<float>("lego_loam/depthFactor", depthFactor, 1000.0);
        nh.param<float>("lego_loam/cameraHeight", cameraHeight, 0.5);
        nh.param<int>("lego_loam/groundScanInd",groundScanInd,241);

        nh.param<bool>("lego_loam/loopClosureEnableFlag", loopClosureEnableFlag, true);
        nh.param<bool>("lego_loam/imuDebug", imuDebug, false);

        nh.param<float>("lego_loam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lego_loam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lego_loam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lego_loam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lego_loam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lego_loam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("lego_loam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lego_loam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lego_loam/extrinsicTrans", extTransV, vector<double>());
//        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
//        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
//        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
//        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("lego_loam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lego_loam/surfThreshold", surfThreshold, 0.1);
        nh.param<float>("lego_loam/groundTh", groundTh, 0.1);
        nh.param<float>("lego_loam/segmentDiff", segmentDiff, 0.01);
        nh.param<int>("lego_loam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lego_loam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lego_loam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lego_loam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lego_loam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lego_loam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lego_loam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lego_loam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lego_loam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lego_loam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lego_loam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lego_loam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lego_loam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lego_loam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("lego_loam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("lego_loam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lego_loam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lego_loam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lego_loam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lego_loam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lego_loam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lego_loam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lego_loam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }
};


#endif
