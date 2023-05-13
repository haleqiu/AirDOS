

#ifndef MAPHUMANTRAJACTORY_H
#define MAPHUMANTRAJACTORY_H

#include<opencv2/core/core.hpp>

#include<mutex>
#include <map>


namespace ORB_SLAM2
{

  class MapPoint;
  class Map;

// The structure of the object segments, including the distance of the estimation.
struct Rigidbody{//segments
  float mnDistance;
  int mnId;
  bool isOptimized = false;
  bool isBad = false;
};

// The trajectory of the human pose includes time, motion and the distance of each part.
// This will be added and associated with the Vertex created in g2o, ex. Vertex Distance.
// The state of the motion is also stored here.
class MapHumanTrajectory
{
public:
    MapHumanTrajectory(Map* pMap, int nHmId);

    std::map<KeyFrame*,size_t> GetObservations();
    void AddMapHumanPose(MapHumanPose* pHumanPose);

    // Get the vector of HumanPose, to avoid increasing vector
    std::vector<MapHumanPose*> GetMapHumanTrajectory();
    MapHumanPose* GetMapHumanPose(const size_t &idx);

public:
  long unsigned int mnId;
  long unsigned int mnTrackID;
  static long unsigned int nNextId;

  cv::Mat mTMotion; // Object Motion model
  std::vector<MapHumanPose*> mvHumanTrajactory;// Sequential
  // std::vector<cv::Mat> mvMotions; TODO useless now
  // std::std::vector<cv::Mat> mvAccerlations;// the second order motion constraint, TODO preserved variables
  std::vector<double> mvTimeStamps;// Record the timestamp for each motion state, TODO useless now

  // The vector with 13 rigid segment on a dynamic human.
  std::vector<Rigidbody> mvRigidBodys;

  long unsigned int mnBALocalForHM;
  long unsigned int mnBAFixedForHM;

  bool isOptimized = false;
  int mnHumanPoses = 0; //the simple counting of how many human poses observed
  float thDistReject = 1; // Predefined threadshold to reject outlier. TODO useless after RANSAC
  int mnBadTrack = 0;

protected:

    // Keyframes observing the point and associated index in keyframe
    // The id is the index for Human Pose in the trajectory
    std::map<KeyFrame*,size_t> mmapObservations;

    // Reference KeyFrame
    std::mutex mMutexMotion; // This is not used TODO
    std::mutex mMutexHumanTrajectory;
    // std::mutex mMutexFeatures;

    Map* mpMap;

};

} //namespace ORB_SLAM

#endif // MAPHUMANTRAJACTORY_H
