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

// Headless MapDrawer for perseus-v3 (nix/extra-packages/patches/orb-slam3): the same
// interface without Pangolin or OpenGL. Tracking still reports the camera pose here every
// frame, so that is kept; everything that drew is a no-op.

#include "MapDrawer.h"

namespace ORB_SLAM3
{

    MapDrawer::MapDrawer(Atlas* pAtlas, const string& strSettingPath, Settings* settings)
        : mpAtlas(pAtlas),
          mKeyFrameSize(0),
          mKeyFrameLineWidth(0),
          mGraphLineWidth(0),
          mPointSize(0),
          mCameraSize(0),
          mCameraLineWidth(0)
    {
        (void)strSettingPath;
        (void)settings;
    }

    void MapDrawer::newParameterLoader(Settings* settings) { (void)settings; }

    bool MapDrawer::ParseViewerParamFile(cv::FileStorage& fSettings)
    {
        (void)fSettings;
        return true;
    }

    void MapDrawer::DrawMapPoints() {}

    void MapDrawer::DrawKeyFrames(const bool, const bool, const bool, const bool) {}

    void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f& Tcw)
    {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.inverse();
    }

    void MapDrawer::SetReferenceKeyFrame(KeyFrame* pKF) { (void)pKF; }

}  // namespace ORB_SLAM3
