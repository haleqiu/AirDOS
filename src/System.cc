/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include "System_utils.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2 {

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer) : mSensor(sensor), mpViewer(static_cast<Viewer *>(nullptr)), mbReset(false),
                                            mbActivateLocalizationMode(false),
                                            mbDeactivateLocalizationMode(false) {
      // Output welcome message
      cout << "Input sensor was set to: ";

      if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
      else if (mSensor == STEREO)
        cout << "Stereo" << endl;
      else if (mSensor == RGBD)
        cout << "RGB-D" << endl;

      //Check settings file
      cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
      if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
      }

      //Load ORB Vocabulary
      cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

      mpVocabulary = new ORBVocabulary();
      bool bVocLoad = false;
      if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
      else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile); // .bin

      if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
      }
      cout << "Vocabulary loaded!" << endl << endl;

      //Create KeyFrame Database
      mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

      //Create the Map
      mpMap = new Map();

      //Create Drawers. These are used by the Viewer
      mpFrameDrawer = new FrameDrawer(mpMap);
      mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

      //Initialize the Tracking thread
      //(it will live in the main thread of execution, the one that called this constructor)
      mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                               mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

      //Initialize the Local Mapping thread and launch
      mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
      mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

      //Initialize the Loop Closing thread and launch
      mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
      mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

      //Initialize the Viewer thread and launch
      if (bUseViewer) {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
      }

      //Set pointers between threads
      mpTracker->SetLocalMapper(mpLocalMapper);
      mpTracker->SetLoopClosing(mpLoopCloser);

      mpLocalMapper->SetTracker(mpTracker);
      mpLocalMapper->SetLoopCloser(mpLoopCloser);

      mpLoopCloser->SetTracker(mpTracker);
      mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, bool bOffLine) : mSensor(sensor), mpViewer(static_cast<Viewer *>(nullptr)),
                                                           mbReset(false), mbActivateLocalizationMode(false),
                                                           mbDeactivateLocalizationMode(false), mbOffLine(bOffLine) {
      cout << "Input sensor was set to: ";

      if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
      else if (mSensor == STEREO)
        cout << "Stereo" << endl;
      else if (mSensor == RGBD)
        cout << "RGB-D" << endl;

      //Check settings file
      cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
      if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
      }

      //Load ORB Vocabulary
      cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

      mpVocabulary = new ORBVocabulary();
      bool bVocLoad = false;
      if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
      else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile); // .bin
      if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
      }
      cout << "Vocabulary loaded!" << endl << endl;

      //Create KeyFrame Database //TODO temp
      mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

      //Create the Map
      mpMap = new Map();

      //Create Drawers. These are used by the Viewer
      mpFrameDrawer = new FrameDrawer(mpMap);
      mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

      //Initialize the Tracking thread
      //(it will live in the main thread of execution, the one that called this constructor)
      if (mbOffLine)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mbOffLine);
      else
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

      //Without Runing this thread
      mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);

      if (!mbOffLine) {
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);

        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpTracker->SetLoopClosing(mpLoopCloser);
      }

      //Initialize the Viewer thread and launch
      if (bUseViewer) {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
      }

      mpTracker->SetLocalMapper(mpLocalMapper);
      mpLocalMapper->SetTracker(mpTracker);

    }

    cv::Mat
    System::TrackStereoHuman(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat &imSegLeft, cv::Mat &imSegRight,
                             const cv::Mat &imDepth, const double &timestamp, const int counter) {
      if (mSensor != STEREO) {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
      }

      // Check mode change
      {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
          mpLocalMapper->RequestStop();

          // Wait until Local Mapping has effectively stopped
          while (!mpLocalMapper->isStopped()) {
            usleep(1000);
          }

          mpTracker->InformOnlyTracking(true);
          mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
          mpTracker->InformOnlyTracking(false);
          mpLocalMapper->Release();
          mbDeactivateLocalizationMode = false;
        }
      }

      // Check reset //TODO: This is weird for me
      {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
          mpTracker->Reset();
          mbReset = false;
        }
      }

      cv::Mat Tcw = mpTracker->GrabImageStereoHuman(imLeft, imRight, imSegLeft, imSegRight, timestamp,
                                                    counter);

      unique_lock<mutex> lock2(mMutexState);
      mTrackingState = mpTracker->mState;
      mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
      mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
      return Tcw;
    }

    cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) {
      if (mSensor != STEREO) {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
      }

      // Check mode change
      {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
          mpLocalMapper->RequestStop();

          // Wait until Local Mapping has effectively stopped
          while (!mpLocalMapper->isStopped()) {
            usleep(1000);
          }

          mpTracker->InformOnlyTracking(true);
          mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
          mpTracker->InformOnlyTracking(false);
          mpLocalMapper->Release();
          mbDeactivateLocalizationMode = false;
        }
      }

      // Check reset
      {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
          mpTracker->Reset();
          mbReset = false;
        }
      }

      cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

      unique_lock<mutex> lock2(mMutexState);
      mTrackingState = mpTracker->mState;
      mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
      mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
      return Tcw;
    }

    void System::ActivateLocalizationMode() {
      unique_lock<mutex> lock(mMutexMode);
      mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
      unique_lock<mutex> lock(mMutexMode);
      mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged() {
      static int n = 0;
      int curn = mpMap->GetLastBigChangeIdx();
      if (n < curn) {
        n = curn;
        return true;
      } else
        return false;
    }

    void System::Reset() {
      unique_lock<mutex> lock(mMutexReset);
      mbReset = true;
    }

    void System::Shutdown() {
      mpLocalMapper->RequestFinish();
      if (!mbOffLine)
        mpLoopCloser->RequestFinish();
      if (mpViewer) {
        mpViewer->RequestFinish();
        while (!mpViewer->isFinished())
          usleep(5000);
      }

      // Wait until all thread have effectively stopped
      if (!mbOffLine)
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
          usleep(5000);
        }

      if (mpViewer)
        pangolin::BindToContext("AirDOS: Map Viewer");
    }

    void System::SaveTrajectoryTUM(const string &filename) {
      cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
      if (mSensor == MONOCULAR) {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
      }

      vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

      // Transform all keyframes so that the first keyframe is at the origin.
      // After a loop closure the first keyframe might not be at the origin.
      cv::Mat Two = vpKFs[0]->GetPoseInverse();

      ofstream f;
      f.open(filename.c_str());
      f << fixed;

      // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
      // We need to get first the keyframe pose and then concatenate the relative transformation.
      // Frames not localized (tracking failure) are not saved.

      // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
      // which is true when tracking failed (lbL).
      list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
      list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
      list<bool>::iterator lbL = mpTracker->mlbLost.begin();
      for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
               lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++) {
        if (*lbL)
          continue;

        KeyFrame *pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while (pKF->isBad()) {
          Trw = Trw * pKF->mTcp;
          pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
          << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
      }
      f.close();
      cout << endl << "trajectory (all) saved!" << endl;
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
      cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

      vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

      // Transform all keyframes so that the first keyframe is at the origin.
      // After a loop closure the first keyframe might not be at the origin.
      //cv::Mat Two = vpKFs[0]->GetPoseInverse();

      ofstream f;
      f.open(filename.c_str());
      f << fixed;

      for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (pKF->isBad())
          continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
          << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

      }

      f.close();
      cout << endl << "trajectory (key) saved!" << endl;
    }

    void System::SaveTrajectoryKITTI(const string &filename) {
      cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
      if (mSensor == MONOCULAR) {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
      }

      vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

      // Transform all keyframes so that the first keyframe is at the origin.
      // After a loop closure the first keyframe might not be at the origin.
      cv::Mat Two = vpKFs[0]->GetPoseInverse();

      ofstream f;
      f.open(filename.c_str());
      f << fixed;

      // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
      // We need to get first the keyframe pose and then concatenate the relative transformation.
      // Frames not localized (tracking failure) are not saved.

      // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
      // which is true when tracking failed (lbL).
      list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
      list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
      for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end();
           lit != lend; lit++, lRit++, lT++) {
        ORB_SLAM2::KeyFrame *pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (pKF->isBad()) {
          //  cout << "bad parent" << endl;
          Trw = Trw * pKF->mTcp;
          pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " "
          << twc.at<float>(0) << " " <<
          Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1)
          << " " <<
          Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2)
          << endl;
      }
      f.close();
      cout << endl << "trajectory saved (kitti) !" << endl;
    }

    int System::GetTrackingState() {
      unique_lock<mutex> lock(mMutexState);
      return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints() {
      unique_lock<mutex> lock(mMutexState);
      return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
      unique_lock<mutex> lock(mMutexState);
      return mTrackedKeyPointsUn;
    }


    void System::ReadAllHumanPoses(int nframes) {
      Eigen::MatrixXd humans;
      for (int i = 0; i < nframes; i++) {
        char frame_index_c[256];
        sprintf(frame_index_c, "%06d", i);
        std::string pose_path_left = folder_path + "/alphapose_0/" + frame_index_c + ".txt";
        if (!read_number_txt(pose_path_left, humans, 54)) {
          humans.resize(0, 54);
        }
        mvAllHumanPosesLeft.push_back(humans);

        std::string pose_path_right = folder_path + "/alphapose_1/" + frame_index_c + ".txt";
        if (!read_number_txt(pose_path_right, humans, 54)) {
          //can't find the pose or
          humans.resize(0, 54);
        }
        mvAllHumanPosesRight.push_back(humans);
      }
    }

    void System::ReadTrackId(int nframes) {
      Eigen::MatrixXd eTrackId;
      for (int i = 0; i < nframes; i++) {
        char frame_index_c[256];
        sprintf(frame_index_c, "%06d", i);
        std::string trackid_path_left = folder_path + "/track_id_alpha/" + frame_index_c + ".txt";
        if (!read_number_txt(trackid_path_left, eTrackId, 1)) {
          //can't find the pose or
          eTrackId.resize(0, 1);
        }
        mvTrackIds.push_back(eTrackId);
      }
    }

    void System::ReadGroundTruthPoses(std::string path, double time_first) {
      Eigen::MatrixXd mGTpose;
      if (!read_number_txt(path, mGTpose, 8)) {
        //can't find the pose or
        mGTpose.resize(0, 8);
      }
      int lens = mGTpose.rows();
      if (lens > 0) {

        int nfirstframe = 0;
        cv::Mat mfirstTcw = cv::Mat::eye(4, 4, CV_32F);
        std::cerr << "time: " << time_first - mGTpose(nfirstframe, 0) << '\n';
        if (time_first > mGTpose(nfirstframe, 0)) {
          while (nfirstframe < lens) {
            if ((time_first - mGTpose(nfirstframe, 0)) < 0.01) {
              break;
            } else
              nfirstframe++;
          }
          std::cerr << "nfirstframe" << nfirstframe << '\n';
          for (int i = nfirstframe; i < lens; i++) {
            Eigen::Quaterniond q(mGTpose(i, 4), mGTpose(i, 5), -mGTpose(i, 6), -mGTpose(i, 7));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            cv::Mat R = Converter::toCvMat(q.normalized().toRotationMatrix());
            Tcw.at<float>(0, 3) = -mGTpose(i, 1);
            Tcw.at<float>(1, 3) = mGTpose(i, 2);
            Tcw.at<float>(2, 3) = mGTpose(i, 3);
            R.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));

            if (i == nfirstframe) {
              mfirstTcw = Tcw.clone();
              // mfirstTcw.at<float>(0,3) = -1 * mfirstTcw.at<float>(0,3);
              // mfirstTcw.at<float>(1,3) = -1 * mfirstTcw.at<float>(1,3);
              // mfirstTcw.at<float>(2,3) = -1 * mfirstTcw.at<float>(2,3);

            }
            std::cerr << mfirstTcw << '\n';
            double time_stamp = mGTpose(i, 0);
            std::pair<int, cv::Mat> mpGTPose;
            mpGTPose.first = time_stamp;
            mpGTPose.second = mfirstTcw.inv() * Tcw;
            std::cerr << mpGTPose.second << '\n';
            mvGTPoss.push_back(mpGTPose);
          }
          mbvisGTpose = true;
        }
      }
    }

    void System::SetDataPath(std::string path) {
      folder_path = path;
    }

    void System::BeforeEnd() {
      cout << "Before End (line " << __LINE__ << "), saving maps" << endl;
      // SAVE Map down
      if (!mpTracker->sMetaDataPath.empty()) {
        mpTracker->SaveMap(mpTracker->sMetaDataPath);
        cerr << "Metadata saved in: " << mpTracker->sMetaDataPath << endl;
      } else
        mpTracker->SaveMap(folder_path + "/metadata");
    }

    void System::BeforeEnd(std::string strSavePath) {
      cout << "Before End (line " << __LINE__ << "), saving maps" << endl;
      mpTracker->sMetaDataPath = std::move(strSavePath);
      // SAVE Map down
      mpTracker->SaveMap(mpTracker->sMetaDataPath);
      cout << "Metadata saved in: " << mpTracker->sMetaDataPath << endl;
    }

    void System::SaveMapPoints(Eigen::MatrixXd &map_points_mat) {
      vector<MapPoint *> pmappoints = mpMap->GetAllMapPoints();
      int num_points = pmappoints.size();

      map_points_mat.resize(num_points, 3);
      int count = 0;
      for (MapPoint *pmp: pmappoints) {
        cv::Mat worldpos = pmp->GetWorldPos();
        map_points_mat(count, 0) = worldpos.at<double>(0);
        map_points_mat(count, 1) = worldpos.at<double>(1);
        map_points_mat(count, 2) = worldpos.at<double>(2);
      }
    }

} //namespace ORB_SLAM
