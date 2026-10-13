/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Headless Viewer for perseus-v3 (nix/extra-packages/patches/orb-slam3): the same
// interface without Pangolin. Nothing constructs one while System is built with
// bUseViewer = false, but Tracking and System still call these, so they must link. A
// Viewer that is never run is permanently finished and stopped, so nothing waits on it.

#include "Viewer.h"

namespace ORB_SLAM3
{

    Viewer::Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking* pTracking,
                   const string& strSettingPath, Settings* settings)
        : both(false),
          mpSystem(pSystem),
          mpFrameDrawer(pFrameDrawer),
          mpMapDrawer(pMapDrawer),
          mpTracker(pTracking),
          mT(0),
          mImageWidth(0),
          mImageHeight(0),
          mImageViewerScale(1),
          mViewpointX(0),
          mViewpointY(0),
          mViewpointZ(0),
          mViewpointF(0),
          mbFinishRequested(false),
          mbFinished(true),
          mbStopped(true),
          mbStopRequested(false),
          mbStopTrack(false)
    {
        (void)strSettingPath;
        (void)settings;
    }

    void Viewer::newParameterLoader(Settings* settings) { (void)settings; }

    bool Viewer::ParseViewerParamFile(cv::FileStorage& fSettings)
    {
        (void)fSettings;
        return true;
    }

    void Viewer::Run() { SetFinish(); }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return true;
    }

    bool Viewer::isStepByStep() { return false; }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = true;
        return true;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
        mbStopRequested = false;
    }

}  // namespace ORB_SLAM3
