#include "MapHumanPose.h"
#include "ORBmatcher.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

  long unsigned int MapHumanPose::nNextId=0;

  MapHumanPose::MapHumanPose(const std::vector<cv::Mat> &vPos, const std::vector<bool> &visbad, KeyFrame *pRefKF, Map* pMap, int nTrackID):
   mnTrackId(nTrackID), mTimeStamp(pRefKF->mTimeStamp), mpRefKF(pRefKF), mpMap(pMap)//, mnFirstKFid(pRefKF->mnId)
  {
    unique_lock<mutex> lock(mMutexHumanPose);
    mnId = nNextId++;

    int count = 0;// so ugly
    mTimeStamp = mpRefKF->mTimeStamp;
    for (std::vector<cv::Mat>::const_iterator itMat = vPos.begin(); itMat != vPos.end(); itMat++){
        MapHumanKey* pMapKey = new MapHumanKey;
        pMapKey->mnId = nNextId++;
        (*itMat).copyTo(pMapKey->WorldPos);
        pMapKey->bIsBad = visbad[count]; count++;
        pMapKey->bOptimized = false;
        mvHumanKeyPos.push_back(pMapKey);
    }

    // set the bad flag through the distance it see
    // Save the segment in each observation, mnTrackID is unique for every HMT
    for (int itr = 0; itr < mpMap->mnbodyparts; itr++){
        HumanKeyPair hmkeypair;
        int id1 = mpMap->body1[itr]; int id2 = mpMap->body2[itr];
        hmkeypair.idFirstKey = mvHumanKeyPos[id1]->mnId; hmkeypair.idSecondKey = mvHumanKeyPos[id2]->mnId;
        hmkeypair.idDistance = itr + mnTrackId*mpMap->mnbodyparts;

        // TODO I don't think this is required any more,
        float distance = cv::norm(vPos[id1], vPos[id2]);
        if (visbad[id1] || visbad[id2]) hmkeypair.bIsBad = true;
        if (distance > 1) hmkeypair.bIsBad = true;
        mvHumanKeyPair.push_back(hmkeypair);
    }

  }

// Almost the same except for the Frame
// But add reference KF
  MapHumanPose::MapHumanPose(const std::vector<cv::Mat> &vPos, const std::vector<bool> &visbad, KeyFrame *pRefKF, Map* pMap, int nTrackID, double dTimeStamp):
  mTimeStamp(dTimeStamp), mpMap(pMap), mnTrackId(nTrackID), mpRefKF(pRefKF)
  {
    unique_lock<mutex> lock(mMutexHumanPose);
    mnId = nNextId++;

    int count = 0;
    for (std::vector<cv::Mat>::const_iterator itMat = vPos.begin(); itMat != vPos.end(); itMat++){
        MapHumanKey* pMapKey = new MapHumanKey;
        pMapKey->mnId = nNextId++;
        (*itMat).copyTo(pMapKey->WorldPos);
        pMapKey->bIsBad = visbad[count]; count++;
        pMapKey->bOptimized = false;
        mvHumanKeyPos.push_back(pMapKey);
    }
    
    // Initialize the HumanKeyPair
    for (int itr = 0; itr < mpMap->mnbodyparts; itr++){
        HumanKeyPair hmkeypair;
        int id1 = mpMap->body1[itr]; int id2 = mpMap->body2[itr];
        hmkeypair.idFirstKey = mvHumanKeyPos[id1]->mnId; hmkeypair.idSecondKey = mvHumanKeyPos[id2]->mnId;
        hmkeypair.idDistance = itr + mnTrackId*mpMap->mnbodyparts;

        float distance = cv::norm(vPos[id1], vPos[id2]);
        if (visbad[id1] || visbad[id2]) hmkeypair.bIsBad = true;
        if (distance > 1) hmkeypair.bIsBad = true; // check the distance
        mvHumanKeyPair.push_back(hmkeypair);
    }

    // MapHumanPose can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  }

  cv::Mat MapHumanPose::GetHumanKeyPos(int nkey)
  {
      unique_lock<mutex> lock(mMutexHumanPose);
      return mvHumanKeyPos[nkey]->WorldPos.clone();
  }

  bool MapHumanPose::isBad(int nkey)
  {
      unique_lock<mutex> lock(mMutexHumanPose);
      return mvHumanKeyPos[nkey]->bIsBad;
  }

  void MapHumanPose::SetHumanKeyPos(int nkey, cv::Mat HumanKeyPos, bool isOptimized){
      unique_lock<mutex> lock(mMutexHumanPose);
      mvHumanKeyPos[nkey]->WorldPos = HumanKeyPos.clone();
      mvHumanKeyPos[nkey]->bOptimized = isOptimized;
  }

  bool MapHumanPose::isOptimized(int nkey)
  {
      unique_lock<mutex> lock(mMutexHumanPose);
      return mvHumanKeyPos[nkey]->bOptimized;
  }

// This will be called in Tracking if the Pose is observed by a key frame
  void MapHumanPose::AddObservation(KeyFrame* pKF, size_t idx)
  {
    unique_lock<mutex> lock(mMutexHumanPose);
    mbIsInKeyFrame = true;
    mObservations.first = pKF;
    mObservations.second = idx; //This id is for associating the stored id in the KeyFrame
  }

  void MapHumanPose::EraseObservation(KeyFrame* pKF)
  {
      isEarsed = true;
      mbIsInKeyFrame = false;
      std::cerr << "earse: "<<mnTrackId << '\n';
  }

} //namespace ORB_SLAM
