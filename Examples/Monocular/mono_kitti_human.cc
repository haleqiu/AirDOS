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


#include<iostream>
#include<algorithm>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImage(const string &strPathToSequence, vector<string> &vstrImageLeft, vector<double> &vTimestamps);

void LoadSeg(const string &strPathToSequence, vector<string> &vstrSegLeft, size_t nTimes);

int main(int argc, char **argv) {
  if (argc != 6 && argc != 5) {
    cerr << endl
         << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_save_trajectory [path_to_save_map?]"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrSegLeft;
  vector<double> vTimestamps;
  LoadImage(string(argv[3]), vstrImageLeft, vTimestamps);
  LoadSeg(string(argv[3]), vstrSegLeft, vTimestamps.size());

  int nImages = (int) vstrImageLeft.size();
  int nStartImage = 0;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  // Here, set the flag OffLine
  std::cout << "System Config path: " << argv[2] << std::endl;
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  int iIsOffline = fsSettings["System.IsOffline"];
  bool bIsOffline = iIsOffline;

  if (bIsOffline) {
    std::cout << "System running in offline mode." << std::endl;
  }

//  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, bIsOffline);
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
  SLAM.folder_path = string(argv[3]);

  // Mannualy set the end point of the sequence,
  int iClip = fsSettings["Schedular.nEndImage"];
  bool isClip = iClip;
  if (isClip) {
    std::cout << "Manually clip current sequence at: " << iClip << '\n';
    if (iClip < nImages) {
      nImages = iClip;
    }
    nStartImage = fsSettings["Schedular.nStartImage"];
  }

  // The flag to determine if using the semantic segmentation
  int iMask = fsSettings["System.IsMask"];
  bool bMask = iMask;

  SLAM.ReadAllHumanPoses(nImages);
  SLAM.ReadTrackId(nImages);

  // Vector for tracking time statistics
  vector<double> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat imLeft, imLeftSeg;

  for (int ni = nStartImage; ni < nImages; ni++) {
//    cout << "Loading: Image from " << vstrImageLeft[ni] << endl;
//    cout << "Loading: Segmentation from " << vstrSegLeft[ni] << endl;
//     Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
    if (bMask) {
      imLeftSeg = cv::imread(vstrSegLeft[ni], cv::IMREAD_GRAYSCALE);
    }

    double tframe = vTimestamps[ni];

    if (imLeft.empty()) {
      cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    SLAM.TrackMonocularHuman(imLeft, imLeftSeg, tframe, ni);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    if (!bIsOffline) {
      // Wait to load the next frame
      double T = 0;
      if (ni < nImages - 1)
        T = vTimestamps[ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestamps[ni - 1];
      std::cout << "time used: " << ttrack << '\n';
      if (ttrack < T)
        usleep((T - ttrack) * 1e6);
    }
  }

  if (argc == 5) SLAM.BeforeEnd();
  else SLAM.BeforeEnd(string(argv[5]));

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  double totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM(argv[4]);
  return 0;
}

/**
 * Fill the path to all image inputs in image_0 folder to vstrImageLeft
 */
void LoadImage(const string &strPathToSequence, vector<string> &vstrImageLeft, vector<double> &vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  string timeString;
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    getline(fTimes, timeString);
    if (timeString.empty()) continue;

    stringstream parser;
    double t;
    parser << timeString;
    parser >> t;
    vTimestamps.push_back(t);
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";
  const size_t nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  for (size_t i = 0; i < nTimes; i++) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
  }
}

/**
 * Fill the path to all segmentation image on left camera (image_0) to vstrSegLeft
 */
void LoadSeg(const string &strPathToSequence, vector<string> &vstrSegLeft, size_t nTimes) {
  string strPrefixLeft = strPathToSequence + "/rcnnseg_image_0/";
  vstrSegLeft.resize(nTimes);
  for (int i = 0; i < nTimes; i++) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrSegLeft[i] = strPrefixLeft + ss.str() + ".png";
  }
}
