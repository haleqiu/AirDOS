

#ifndef MAPHUMANPOSE_H
#define MAPHUMANPOSE_H

#include<opencv2/core/core.hpp>

#include<mutex>
#include <map>
#include <Eigen/Dense>

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;
class Map;
class Frame;
class MapHumanTrajectory;

// the struct to store human pose consider for a new class or not
struct human_pose{
  int human_idx;
  std::vector<cv::KeyPoint> vHumanKeyPoints;
  std::vector<cv::KeyPoint> vHumanKeyPointsRight;
  std::vector<float> vKeysConfidence;
  std::vector<float> vKeysConfidenceRight;
  std::vector<float> vHumanKeyDepths;
  std::vector<bool> vbIsBad;
};

// TODO update the MapHumanKey according to the camera pose
struct MapHumanKey{
  int mnId;
  cv::Mat WorldPos;
  cv::Mat RelativePose;
  bool bIsBad = false; // Bad Flag for estimation
  bool bIsLost = false; // Lost Flag for motion tracking
  bool bOptimized = false;
  // if the rigidity constraint not work for you, then this observation is problematic
  bool bIsFirstBad = false;
  bool bIsSecondBad = false;
};

// This store the key point segment
struct HumanKeyPair{
  int idFirstKey;
  int idSecondKey;
  int idDistance;
  bool bIsBad = false;
  bool bOptimized = false;
};

// The design of this class is to store the human key point.
// One more thing to store is the semantic label for the human pose.
class MapHumanPose
{
public:
    MapHumanPose(const std::vector<cv::Mat> &vPos, const std::vector<bool> &visbad, KeyFrame* pRefKF, Map* pMap, int nTrackID);
    MapHumanPose(const std::vector<cv::Mat> &vPos, const std::vector<bool> &visbad, KeyFrame* pRefKF, Map* pMap, int nTrackID, double dTimeStamp);

    void SetHumanKeyPos(int nkey, cv::Mat HumanKeyPos, bool isOptimized);
    cv::Mat GetHumanKeyPos(int nkey);

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    bool isBad(int nkey);
    bool isOptimized(int nkey);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;

    long unsigned int mnTrackId = -1;

    // TODO move it protected
    // the key pair stored for rigidity constraint
    std::vector<HumanKeyPair> mvHumanKeyPair;
    // human key directly store the pose of each key joints
    std::vector<MapHumanKey*> mvHumanKeyPos;

    // TimeStamp when the human pose been observed
    double mTimeStamp;
    // Keyframes observing the human pose and associated index in keyframe
    // Since only keyframe associated, then use pair instead
    std::pair<KeyFrame*,size_t> mObservations;

    bool mbIsInKeyFrame = false; // This is for adding the human pose that is not in keyframe
    bool isLost = false;
    bool isEarsed = false; // TODO this is not implemented

    MapHumanTrajectory* mpRefHMT;
    // Reference KeyFrame // TODO what if all the viewed frame.
    KeyFrame* mpRefKF;


protected:

    std::vector<int> vnKeyRight;


    Map* mpMap;

    std::mutex mMutexHumanPose;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
