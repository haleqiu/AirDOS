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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::AddMapHumanPose(MapHumanPose *pMapHumanPose)
{
    unique_lock<mutex> lock(mMutexMap);
    mspHumanPose.insert(pMapHumanPose);
}

// Get the mutex from map with mutex
MapHumanTrajectory* Map::GetMapHumanTrajectory(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexMap);
    std::map<int, MapHumanTrajectory*>::iterator ptHMT = mmapHumanTrajectory.find(idx);
    if (ptHMT != mmapHumanTrajectory.end())
      return ptHMT->second;
    else
      return NULL;
}

// insert the hmt into the set and also insert to the map
void Map::AddMapHumanTrajectory(MapHumanTrajectory *pMapHumanTrajectory, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexMap);
    mspHumanTrajactory.insert(pMapHumanTrajectory);
    mmapHumanTrajectory[idx] = pMapHumanTrajectory;
}

void Map::EraseMapHumanTrajactory(MapHumanTrajectory *pMapHumanTrajectory){
    unique_lock<mutex> lock(mMutexMap);
    mspHumanTrajactory.erase(pMapHumanTrajectory);
    mmapHumanTrajectory.erase(pMapHumanTrajectory->mnTrackID); // TODO It is still in the memory
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapHumanPose*> Map::GetAllMapHumanPoses()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapHumanPose*>(mspHumanPose.begin(),mspHumanPose.end());
}

// TODO filter out the bad trajectory and return the optimized trajectory
vector<MapHumanTrajectory*> Map::GetAllMapHumanTrajactory(bool bisOptimized)
{
    unique_lock<mutex> lock(mMutexMap);
    if (bisOptimized)
      {
        // TODO reserve the size
        vector<MapHumanTrajectory*> vpHMT;
        for (MapHumanTrajectory* pHMT : mspHumanTrajactory)
        {
          if (pHMT->isOptimized)
            vpHMT.push_back(pHMT);
        }
        return vpHMT;
      }
    else
      return vector<MapHumanTrajectory*>(mspHumanTrajactory.begin(),mspHumanTrajactory.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

// Shall we assign the id according to an order?
int Map::FindColor(int nseg){
  int colorid;
    if (mapColorMap.count(nseg)>0){
        colorid = mapColorMap.find(nseg)->second;
    }
    else{
        colorid = nseg%mColorIdxMap.size();
        mapColorMap.insert(std::pair<int,int>(nseg, colorid));
    }
    return colorid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mspHumanTrajactory.clear();
    mmapHumanTrajectory.clear();
    mspHumanPose.clear();
    mvCurrentTrackID.clear(); //this should also been clear or we will try to find it in Drawer
}

} //namespace ORB_SLAM
