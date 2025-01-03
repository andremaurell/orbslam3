#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>
#include<Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"
#include <ros/ros.h>

namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

    // Tracking functions
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Localization modes
    void ActivateLocalizationMode();
    void DeactivateLocalizationMode();

    // System control
    bool MapChanged();
    void Reset();
    void ResetActiveMap();
    void Shutdown();
    bool isShutDown();

    // Saving trajectory functions
    void SaveTrajectoryTUM(const string &filename);
    void SaveKeyFrameTrajectoryTUM(const string &filename);
    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);
    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveDebugData(const int &iniIdx);
    void SaveTrajectoryKITTI(const string &filename);

    // Tracking info
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();
    void ChangeDataset();
    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

    // **New functions for ArUco integration**
    void SetArucoPose(const Sophus::SE3f &pose); // Set pose detected by ArUco
    void EnableArucoRelocalization(bool enable); // Enable or disable ArUco-based relocalization

    // **New callback function declaration**
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); // Callback to process ArUco poses

private:
    ros::Subscriber mPoseSubscriber;
    void SaveAtlas(int type);
    bool LoadAtlas(int type);
    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary and keyframe database
    ORBVocabulary* mpVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map management
    Atlas* mpAtlas;

    // Tracking and mapping threads
    Tracking* mpTracker;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;

    // Visualization
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // System state
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;
    bool mbShutDown;

    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;
    string mStrVocabularyFilePath;
    Settings* settings_;

    // **Variables for ArUco-based relocalization**
    std::mutex mMutexAruco;
    Sophus::SE3f mCurrentArucoPose; // Pose estimated from ArUco
    bool mbRelocalized; // Flag to indicate relocalization using ArUco
};

} // namespace ORB_SLAM3

#endif // SYSTEM_H
