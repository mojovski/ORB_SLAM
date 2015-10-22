/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_TRACE_H
#define FRAME_TRACE_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM
{

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;

/**
@brief
This class helps to remember the frames which have not been considered as
KeyFrames.
They are also stored for the case the whole map must be exported.
It stores its 
* Pose relative to a KeyFrame
* List of observations, which also interconnect to other FrameTraces!

* Interior camera geometry
* Identifier of the origin image file

*/

class FrameTrace
{
public:
    FrameTrace();
    FrameTrace(const Frame &frame);

    
    // FrameTrace timestamp
    double mTimeStamp;

    // Calibration Matrix and k1,k2,p1,p2 Distortion Parameters
    cv::Mat mK;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;

    // MapPoints associated to keypoints, NULL pointer if not association
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations
    std::vector<bool> mvbOutlier;


    // Camera Pose in the keyframe, ck=camera to key frame
    cv::Mat mTck;

    KeyFrame* mpReferenceKF;


    std::string mROSID; //The ID inside the ROS Package header. http://docs.ros.org/api/std_msgs/html/msg/Header.html

    void ResetRelativeKfPose();

private:
    void UpdatePoseMatrices();

    // Call UpdatePoseMatrices(), before using
    cv::Mat mOk;
    cv::Mat mRck;
    cv::Mat mtck;
    
};

}// namespace ORB_SLAM

#endif // FRAME_H
https://github.com/ros/catkin/issues/694