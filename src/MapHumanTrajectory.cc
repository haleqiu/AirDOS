#include "MapHumanPose.h"
#include "MapHumanTrajectory.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{
  long unsigned int MapHumanTrajectory::nNextId=0;

  MapHumanTrajectory::MapHumanTrajectory(Map* pMap, int nTrackID): mpMap(pMap), mnTrackID(nTrackID)
  {
      unique_lock<mutex> lock(mMutexHumanTrajectory);
      mnId = nNextId++;
      mTMotion = cv::Mat::eye(4,4,CV_32F);

      // Initiate the humans' segments
      for(int itr = 0; itr < mpMap->mnbodyparts; itr++)
      {
          Rigidbody Rigid;
          Rigid.mnId = itr + mnTrackID * mpMap->mnbodyparts;
          mvRigidBodys.push_back(Rigid);
      }
  }

  void MapHumanTrajectory::AddMapHumanPose(MapHumanPose* pHumanPose)
  {
      // The HumanPose have 18 key points for open pose
      unique_lock<mutex> lock(mMutexHumanTrajectory);

      cv::Mat MatMotion = cv::Mat::eye(4,4,CV_32F);
      cv::Mat LastMatPosition;
      cv::Mat CurrentMatPosition;

      mvHumanTrajactory.push_back(pHumanPose);
      mvTimeStamps.push_back(pHumanPose->mTimeStamp);
      pHumanPose->mpRefHMT = this;

      // if it is keyframe insert the observation.
      // this map would be useful in local bundle adjustmet
      if (pHumanPose->mbIsInKeyFrame){
          mmapObservations.insert(pHumanPose->mObservations);
      }

      // initiate the Rigid Body constraint
      if (mnHumanPoses == 0)
      {
          for(int itr = 0; itr < mpMap->mnbodyparts; itr++)
          {
              int id1 = mpMap->body1[itr]; int id2 = mpMap->body2[itr];
              cv::Mat pose1 = pHumanPose->GetHumanKeyPos(id1);
              cv::Mat pose2 = pHumanPose->GetHumanKeyPos(id2);

              // If the initial estimation is too large, simply set it to 0
              double distance = cv::norm(pose1, pose2);
              if (distance >thDistReject)
                  mvRigidBodys[itr].mnDistance = 0;
              else
                  mvRigidBodys[itr].mnDistance = (float)distance;
          }
      }
      mnHumanPoses++;
  }

  // Request for whole vector of the human poses trajectory
  std::vector<MapHumanPose*> MapHumanTrajectory::GetMapHumanTrajectory()
  {
    unique_lock<mutex> lock(mMutexHumanTrajectory);
    return mvHumanTrajactory;
  }

  MapHumanPose* MapHumanTrajectory::GetMapHumanPose(const size_t &idx)
  {
    unique_lock<mutex> lock(mMutexHumanTrajectory);
    return mvHumanTrajactory[idx];
  }

  std::map<KeyFrame*,size_t> MapHumanTrajectory::GetObservations()
  {
    unique_lock<mutex> lock(mMutexHumanTrajectory);
    return mmapObservations;
  }

} //namespace ORB_SLAM
