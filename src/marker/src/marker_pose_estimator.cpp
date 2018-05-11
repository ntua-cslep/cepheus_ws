/*
 * Copyright (c) 2015 GMV. Property of GMV; all rights reserved 
 */

#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>
#include <aruco/highlyreliablemarkers.h>
#include "marker_pose_estimator.h"

MarkerPoseEstimator::MarkerPoseEstimator()
{
    _detector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
    _detector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
    _detector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
}

bool MarkerPoseEstimator::loadDictionary(const std::string &filename)
{
    return aruco::HighlyReliableMarkers::loadDictionary(filename);
}

bool MarkerPoseEstimator::loadBoardParameters(const std::string &filename)
{
    std::ifstream is(filename.c_str());
    if (!is.good())
        return false;

    _board.clear();

    // Skip first line (header)
    std::string line;
    std::getline(is, line);

    int id;
    cv::Point2f t;
    float z;
    cv::Matx22f R;
    cv::Matx43f pts;

    float angle;
    float width;
    while (!is.eof())
    {
        std::getline(is, line);
        if (line.empty())
            continue;
        std::istringstream iss(line);
        iss >> id;
        iss >> t.x >> t.y >> z;
        iss >> width;
        iss >> angle;

        angle += 90.0;
        angle = static_cast<float>(angle / 180.0f * CV_PI);
        float ca = std::cos(angle);
        float sa = std::sin(angle);
        R(0, 0) = ca; R(0, 1) = -sa;
        R(1, 0) = sa; R(1, 1) = ca;

        float half_w = width / 2.0f;
        cv::Point2f pt3D;
        pt3D = R * cv::Point2f(-half_w,  half_w) + t;
        pts(0, 0) = pt3D.x; pts(0, 1) = pt3D.y; pts(0, 2)= z;
        pt3D = R * cv::Point2f( half_w,  half_w) + t;
        pts(1, 0) = pt3D.x; pts(1, 1) = pt3D.y; pts(1, 2)= z;
        pt3D = R * cv::Point2f( half_w, -half_w) + t;
        pts(2, 0) = pt3D.x; pts(2, 1) = pt3D.y; pts(2, 2)= z;
        pt3D = R * cv::Point2f(-half_w, -half_w) + t;
        pts(3, 0) = pt3D.x; pts(3, 1) = pt3D.y; pts(3, 2)= z;
        
        _board[id] = pts;
    }

    return true;
}

void MarkerPoseEstimator::setThresholdParams(float th1, float th2)
{
    _detector.setThresholdParams(th1, th2);
}

void MarkerPoseEstimator::setMinMaxSize(float mn, float mx)
{
    _detector.setMinMaxSize(mn, mx);
}

bool MarkerPoseEstimator::loadCameraParameters(const std::string &filename)
{
    try
    {
        _camParams.readFromXMLFile(filename);
    }
    catch (...)
    {
        return false;
    }

    return true;
}
    
int MarkerPoseEstimator::run(const cv::Mat &img, cv::Mat &R, cv::Mat &T)
{
    _detector.detect(img, _markers);
    if (!_markers.size())
    {
        R = cv::Mat::zeros(3, 3, CV_64F);
        T = cv::Mat::zeros(3, 1, CV_64F);
        return 0;
    }

    std::vector<cv::Point3f> objPoints;
    std::vector<cv::Point2f> imagePoints;
    for (size_t i = 0; i < _markers.size(); i++)
    {
        int id = _markers[i].id;
        markerInfo &mi = _board[id];
        for (size_t j = 0; j < 4; j++)
        {
            imagePoints.push_back(_markers[i][j]);
            objPoints.push_back(cv::Point3f(mi(j, 0), mi(j, 1), mi(j, 2)));
        }
    }

    cv::Mat rvec, tvec, Rvec;
    cv::solvePnP(objPoints, imagePoints, _camParams.CameraMatrix, _camParams.Distorsion, rvec, tvec, false);

    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(objPoints, rvec, tvec, _camParams.CameraMatrix, _camParams.Distorsion, reprojected);
    _reprojError = 0;
    for (size_t i = 0; i < reprojected.size(); i++)
        _reprojError += cv::norm(reprojected[i] - imagePoints[i]);
    _reprojError /= reprojected.size();

    cv::Rodrigues(rvec, Rvec);

    R = Rvec.t();
    T = -R * tvec;

    return static_cast<int>(_markers.size());
}

void MarkerPoseEstimator::drawMarkers(const cv::Mat &img, cv::Mat &outImg) const
{
    img.copyTo(outImg);
    for (size_t i = 0; i < _markers.size(); i++)
    {
        _markers[i].draw(outImg, CV_RGB(0, 0, 255));
    }
}

double MarkerPoseEstimator::getReprojectionError() const
{
    return _reprojError;
}
