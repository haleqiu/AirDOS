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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "MapHumanPose.h"
#include "MapHumanTrajectory.h"
#include <set>
#include <Eigen/Dense>

#include <mutex>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>



namespace ORB_SLAM2
{

class MapHumanPose;
class MapPoint;
class KeyFrame;
class MapHumanTrajectory;

class Map
{
public:
    // The main skleton is design for the motion constriant.
    int mainskleton[5] = {1,2,5,11,8};
    int head[1] = {1};
    int mimainskleton = 5;

    // body1 and body 2 denotes the start and end pose of the human segment
    int mnbodyparts = 14;
    int body1[14] = {1, 1, 8, 2, 5, 2, 3, 5, 6, 8, 9, 11, 12, 1 };
    int body2[14] = {2, 5, 11, 8, 11, 3, 4, 6, 7, 9, 10, 12, 13, 0};

    // The color map for human pose
    std::vector<Eigen::Vector3f> colormap{ // = {0,0,0,0,0,0,1,1,1,1,1,1,2};
    Eigen::Vector3f(0,230,230)/255.0,
    Eigen::Vector3f(0,230,230)/255.0,
    Eigen::Vector3f(0,230,230)/255.0,
    Eigen::Vector3f(0,230,230)/255.0,
    Eigen::Vector3f(0,230,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(230,0,230)/255.0};


    std::vector<Eigen::Vector3f> mColorIdxMap{
    Eigen::Vector3f(230, 0, 0)/255.0,  // red  0
    Eigen::Vector3f(60, 180, 75)/255.0,  // green  1
    Eigen::Vector3f(0, 0, 255)/255.0,	// blue  2
    Eigen::Vector3f(255, 0, 255)/255.0,	// Magenta  3
    Eigen::Vector3f(255, 165, 0)/255.0,   // orange 4
    Eigen::Vector3f(128, 0, 128)/255.0,   //purple 5
    Eigen::Vector3f(0, 255, 255)/255.0,   //cyan 6
    Eigen::Vector3f(210, 245, 60)/255.0,   //lime  7
    Eigen::Vector3f(250, 190, 190)/255.0,//pink  8
    Eigen::Vector3f(0, 128, 128)/255.0};  //Teal  9

    // A simple set for the optimized human trajectory, record the unique trakID
    std::set<int> msetOptimizedTrackID;
    //The trackID that seen in current frame, which will be visualized in MapDrawer.
    //This will always been overwrited
    std::vector<int> mvCurrentTrackID;
    //
    std::map<int, MapHumanTrajectory*> mmapHumanTrajectory;

    std::map<int, int> mapColorMap;
    bool mbIsSeg = false;
    bool mbVOOnlyFlag = false;
    bool mbIsStaticOnly = false;
    float thLongTrajectory = 3;

public:
    Map();

    int FindColor(int nseg);

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    //MapHumanPose
    void AddMapHumanPose(MapHumanPose *pMapHumanPose);
    void AddMapHumanTrajectory(MapHumanTrajectory *pMapHumanTrajectory, const size_t &idx);
    void EraseMapHumanTrajactory(MapHumanTrajectory *pMapHumanTrajectory);
    MapHumanTrajectory* GetMapHumanTrajectory(const size_t &idx);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<MapHumanPose*> GetAllMapHumanPoses();
    std::vector<MapHumanTrajectory*> GetAllMapHumanTrajactory(bool bisOptimized);

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapHumanPose*> mspHumanPose;// TODO consider the human pose in class and pointer in the future
    std::set<MapHumanTrajectory*> mspHumanTrajactory; //a map? TODO seems useless

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
