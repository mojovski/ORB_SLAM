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

#include "FrameTrace.h"
#include "Converter.h"

#include <ros/ros.h>

namespace ORB_SLAM
{

FrameTrace::FrameTrace(const Frame &frame):mTimeStamp(frame.mTimeStamp),
    mK(frame.mK), mDistCoef(frame.mDistCoef), N(frame.N), mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeyUn),
    mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), 
    mfGridElementWidthInv(frame.mfGridElementWidthInv), mfGridElementHeightInv(frame.mfGridElementHeightInv),
    mGrid(frame.mGrid), mpReferenceKF(frame.mpReferenceKF), mROSID(frame.getROSID()),
    mvbOutlier(frame.mvbOutlier)
{
    mTck=frame.mTcw.inv()*mpReferenceKF.mTcw;
}

void FrameTrace::UpdatePoseMatrices()
{ 
    mRck = mTck.rowRange(0,3).colRange(0,3);
    mtck = mTck.rowRange(0,3).col(3);
    mOk = -mRck.t()*mtck;
}

void FrameTrace::void ResetRelativeKfPose()
{
    mTck= Mat::eye(mpReferenceKF.mTcw.rows, mpReferenceKF.mTcw.cols, CV_64F);
}

} //namespace ORB_SLAM
