/*
 * Copyright (c) 2015 GMV. Property of GMV; all rights reserved 
 */

#ifndef __MARKER_POSE_ESTIMATOR_H__
#define __MARKER_POSE_ESTIMATOR_H__

#include <map>
#include <boost/tuple/tuple.hpp>
#include <aruco/cameraparameters.h>
#include <aruco/markerdetector.h>

class MarkerPoseEstimator
{
    typedef cv::Matx43f markerInfo;

public:
    MarkerPoseEstimator();

    bool loadDictionary(const std::string &filename);
    bool loadBoardParameters(const std::string &filename);
    bool loadCameraParameters(const std::string &filename);

    void setThresholdParams(float th1, float th2);
    void setMinMaxSize(float mn, float mx);

    int run(const cv::Mat &img, cv::Mat &R, cv::Mat &T);
    double getReprojectionError() const;
    void drawMarkers(const cv::Mat &img, cv::Mat &outImg) const;

private:
    aruco::CameraParameters _camParams;
    
    aruco::MarkerDetector _detector;
    std::vector<aruco::Marker> _markers;
    std::map<int, markerInfo> _board;
    double _reprojError;
};

#endif /* __MARKER_POSE_ESTIMATOR_H__ */
