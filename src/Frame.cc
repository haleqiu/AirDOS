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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "MapHumanPose.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
= default;

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,const cv::Mat &imSegLeft, const cv::Mat &imSegRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
             Eigen::MatrixXd left_human_poses, Eigen::MatrixXd right_human_poses, std::vector<float> mvSettingHumanTh, Eigen::MatrixXd eTrackIds)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL)),meHumanPosesLeft(left_human_poses),meHumanPosesRight(right_human_poses), meTrackIds(eTrackIds)
{
    // Frame ID
    mnId=nNextId++;
    mimSegLeft = imSegLeft.clone();
    mimSegRight = imSegRight.clone();

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    // This is not elegant
    // if (!imSegLeft.empty())
    //     AddMatchingLable(imSegLeft.clone(), imSegRight.clone());
    // else isSeg = false;
    if (imSegLeft.empty())
        isSeg = false;

    N = (int) mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
        mb = mbf/fx;
    }

    //TODO human pose undistorted
    RejectTh = mvSettingHumanTh[0];

    //TODO, is it really need a clone?
    ComputeHumanPoseTriangulation(imLeft.clone(), imRight.clone());

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    AssignFeaturesToGrid();
}


// Monocular Human Frame Constructor
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imSegLeft, const double &timeStamp, ORBextractor* extractorLeft, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float thDepth,
          Eigen::MatrixXd left_human_poses, std::vector<float> mvSettingHumanTh, Eigen::MatrixXd eTrackIds)
  :mpORBvocabulary(voc),
   mpORBextractorLeft(extractorLeft),
   mpORBextractorRight(static_cast<ORBextractor*>(nullptr)),
   mTimeStamp(timeStamp),
   mK(K.clone()),
   mDistCoef(distCoef.clone()),
   mbf(bf),
   mThDepth(thDepth),
   mnId(nNextId ++),
   mpReferenceKF(static_cast<KeyFrame*>(nullptr)),
   meHumanPosesLeft(left_human_poses),
   meTrackIds(eTrackIds),
   mimSegLeft(imSegLeft.clone())
//   meHumanPosesRight(static_cast<Eigen::MatrixXd>(nullptr)) // FIXME: How to deal with unused field here? (Jan 26)
{
  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft -> GetLevels();
  mfScaleFactor = mpORBextractorLeft -> GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  ExtractORB(0, imLeft);

  isSeg = ! imSegLeft.empty();
  N = (int) mvKeys.size();

  if(mvKeys.empty()) return;

  UndistortKeyPoints();

  // Set no stereo information
  mvuRight = vector<float>(N,-1);
  mvDepth = vector<float>(N,-1);

  mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
  mvbOutlier = vector<bool>(N,false);

  // This is done only for the first Frame (or after a change in the calibration)
  if(mbInitialComputations)
  {
    ComputeImageBounds(imLeft);

    mfGridElementWidthInv =static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
    mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;

    mbInitialComputations=false;
  }

  RejectTh = mvSettingHumanTh[0];
  mb = mbf/fx;

  // FIXME: How to compute the depth of humanpose with one single frame?
  //  without depth information.

  // Write an alternative for ComputeHumanTriangulation
  mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
  mvbOutlier   = vector<bool>(N,false);

  AssignFeaturesToGrid();
}


// Matching left and right
// Temp set up associating left and right human pose
int Frame::MatchingHumanPoses(Eigen::VectorXd Left, Eigen::MatrixXd Right, cv::Mat UFlow, cv::Mat VFlow){
  int mainskleton[5] = {1,2,5,11,8};
  float thMinDist = 30;
  float mindist = 50;
  float minid = -1;

  for (int rid = 0; rid<Right.rows(); rid++){
    double dist = 0; int mcount = 0;

    for(int i = 0; i < 5; i++){
      int hpidx = mainskleton[i];
      double nlscore; nlscore = Left(hpidx*3+2);
      int ul,vl; ul = Left(hpidx*3); vl = Left(hpidx*3+1);

      double nrscore; nrscore = Right.row(rid)(hpidx*3+2);
      double ur,vr; ur = Right.row(rid)(hpidx*3); vr = Right.row(rid)(hpidx*3+1);

      // This part check for bounds , sometimes it do have some strange case
      if ((nlscore <RejectTh) && (nrscore < RejectTh)) continue;
      if (((ul > mnMaxX) || (ul < mnMinX)) ||  ((vl > mnMaxY) || (vl < mnMinY))) continue;

      double fDistanceX = ul - UFlow.at<float>(vl, ul) -ur;
      double fDistanceY = vl - VFlow.at<float>(vl, ul) -vr;

      dist = dist + std::sqrt(fDistanceX*fDistanceX + fDistanceY*fDistanceY); mcount++;
    }
    if (mcount > 0) dist = dist/mcount;
    else continue;
    if (dist < mindist){mindist = dist; minid = rid;}
  }

  if (mindist >= thMinDist){
     minid = -1;
  }
  return (int) minid;
}

void Frame::ComputeHumanPoseDepth(cv::Mat imDepth)
{   
    // Check if there are observation of human
    mnHumanObserved = meHumanPosesLeft.rows();
    if (mnHumanObserved < 1)
    {
        std::cerr << "no human poes readed" << '\n';
        return; 
    }

    int removed_hp = 0;//count if there are bad tracked human pose
    for (int nHumanid = 0; nHumanid < mnHumanObserved; nHumanid++)
    {
        human_pose mhuman;
        if (meTrackIds.rows() > 0)
        {   
            // Bad Human Tracking
            if (meTrackIds(nHumanid,0) <0)
            {
                removed_hp ++;
                continue;
            }
            mhuman.human_idx = meTrackIds(nHumanid,0);
        }
        // If there not track id given
        else 
            mhuman.human_idx = -1;
        
        // TODO prior knowledge that we have 18 human key pose
        for (int i = 0; i<18; i++)
        {
            float u,v; u = meHumanPosesLeft.row(nHumanid)(i*3); v = meHumanPosesLeft.row(nHumanid)(i*3+1);
            float d = imDepth.at<float>(v,u);

            bool bIsHPBad = false;// The Flag to indicate a bad KeyPo
            
            cv::KeyPoint mhumankeys, mhumankeysright;
            mhumankeys.pt.x = u; mhumankeys.pt.y = v;
            // calculate a pseudo right keypoint u-mbf/d mbf/(left-right)) = depth; 
            mhumankeysright.pt.x = u-mbf/d; mhumankeysright.pt.y = v;
            // std::cerr << "d" << d <<std::endl;
            if (d<0.01)
            {
                bIsHPBad = true; // d < 0
                d = 0.01;
            }

            // store the score of the human pose estimation
            float nlscore = meHumanPosesLeft.row(nHumanid)(i*3+2);
            if (nlscore<RejectTh)
                bIsHPBad = true;

            mhuman.vKeysConfidence.push_back(nlscore);
            mhuman.vKeysConfidenceRight.push_back(1);
            mhuman.vHumanKeyPoints.push_back(mhumankeys);
            mhuman.vHumanKeyPointsRight.push_back(mhumankeysright);
            mhuman.vbIsBad.push_back(bIsHPBad);
            mhuman.vHumanKeyDepths.push_back(d);//TODO what if d really < 0
        }
        mvHumanPoses.push_back(mhuman);
    }
    mnHumanObserved = mnHumanObserved - removed_hp; // TODO it is ugly
}

void Frame::ComputeHumanPoseTriangulation(cv::Mat imLeft, cv::Mat imRight)
{
  int nHumansLeft = meHumanPosesLeft.rows();
  int nHumansRight = meHumanPosesRight.rows();
  if (nHumansLeft > nHumansRight) mnHumanObserved = nHumansRight;
  else mnHumanObserved = nHumansLeft;

  // Temp set up for calculating the disparity
  int numberOfDisparities = 48, SADWindowSize = 11;
  int uniquenessRatio = 15, speckleWindowSize = 50, speckleRange = 32;
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, numberOfDisparities, SADWindowSize);
  int cn = imLeft.channels();
  sgbm->setP1(8 * cn * SADWindowSize * SADWindowSize);
  sgbm->setP2(32 * cn * SADWindowSize * SADWindowSize);
  sgbm->setPreFilterCap(63);
  sgbm->setUniquenessRatio(uniquenessRatio);
  sgbm->setSpeckleWindowSize(speckleWindowSize);
  sgbm->setSpeckleRange(speckleRange);
  sgbm->setDisp12MaxDiff(1);

  // Temp Calculate disparity
  cv::Mat disp;
  sgbm->compute(imLeft, imRight, disp);
  disp.convertTo(disp, CV_32F, 1.0 / 16);

//   cv::Mat rightdisp;
//   sgbm->setMinDisparity(-numberOfDisparities);
//   sgbm->setNumDisparities(numberOfDisparities);
//   sgbm->compute(imRight,imLeft,rightdisp);
//   rightdisp.convertTo(rightdisp, CV_32F, 1.0/16);

  if (nHumansLeft < 1 || nHumansRight < 1){
    std::cerr << "no human poes readed" << '\n';
    return;
  }
  else{

    int removed_hp = 0; //TODO if there are no left and right matching
    for (int nHumanid = 0; nHumanid < mnHumanObserved; nHumanid++){
      //left human
      human_pose mhuman;
      if (meTrackIds.rows() > 0)
      {
        if (meTrackIds(nHumanid,0) <0)
        {
            removed_hp ++;
            continue; // Special cases that this pose is bad, can't be tracked
        }
        mhuman.human_idx = meTrackIds(nHumanid,0);
      }
      else
        mhuman.human_idx = -1; // Let the -1 to indicate a bad associated human pose or not tracked

      Eigen::VectorXd eVecLeft = meHumanPosesLeft.row(nHumanid);
      cv::Mat matVdip = cv::Mat(imLeft.rows, imLeft.cols, CV_64F, float(0));
      int iMatchId = MatchingHumanPoses(eVecLeft, meHumanPosesRight, disp, matVdip);
      // iMatchId is index for the human pose in right frame; -1 means no good matching found
      if (iMatchId < 0) {
       removed_hp++; continue; }
      // TODO the 18 here should have a parameters
      for (int i =0;i<18;i++){
        //TODO there may have a matching issue between left and right
        float nlscore; nlscore = meHumanPosesLeft.row(nHumanid)(i*3+2);
        float nrscore; nrscore = meHumanPosesRight.row(iMatchId)(i*3+2);

        int u,v; u = meHumanPosesLeft.row(nHumanid)(i*3); v = meHumanPosesLeft.row(nHumanid)(i*3+1);

        cv::KeyPoint mhumankeys;
        mhumankeys.pt.x = u; mhumankeys.pt.y = v;

        u = meHumanPosesRight.row(iMatchId)(i*3); //v = meHumanPosesRight.row(0)(i*3+1);//stereo matching y should be the same

        cv::KeyPoint mhumankeysright;
        mhumankeysright.pt.x = u; mhumankeysright.pt.y = v;

        bool mbHPBad = false;
        if ((nlscore <RejectTh) && (nrscore < RejectTh))
          mbHPBad = true;

        mhuman.vHumanKeyPoints.push_back(mhumankeys);
        mhuman.vKeysConfidence.push_back(nlscore);
        mhuman.vHumanKeyPointsRight.push_back(mhumankeysright);
        mhuman.vKeysConfidenceRight.push_back(nrscore);

        float disparity = mhumankeys.pt.x - mhumankeysright.pt.x;//u-uR
        if (disparity <= 0) {
          disparity = 0.01;
          mbHPBad = true;
        }
        
        // TODO how we handle the problem of the distance
        if ((mbf/disparity)>20){ 
          mbHPBad = true; //TODO temp set up
        }

        mhuman.vbIsBad.push_back(mbHPBad);
        mhuman.vHumanKeyDepths.push_back(mbf/disparity);

      }
      mvHumanPoses.push_back(mhuman);
    }
    mnHumanObserved = mnHumanObserved - removed_hp; // TODO it is ugly
  }
}

// Frame constructor for stereo vision
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

// Frame constructor for monocular vision
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    cv::Mat mask = cv::Mat(im.size(),CV_8U, cv::Scalar(255));

    if(flag==0){
      if (!mimSegLeft.empty()){ //If There is mask
            mimSegLeft.convertTo(mask,CV_8U);
            mask.setTo(255,mask==0);
            mask.setTo(0,mask==1);
          }
        (*mpORBextractorLeft)(im,mask,mvKeys,mDescriptors);
      }
    else{
      if (!mimSegRight.empty()){ //If There is mask
            mimSegRight.convertTo(mask,CV_8U);
            mask.setTo(255,mask==0);
            mask.setTo(0,mask==1);
          }
        (*mpORBextractorRight)(im,mask,mvKeysRight,mDescriptorsRight);
      }
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::AddMatchingLable(cv::Mat imSegLeft, cv::Mat imSegRight)
{
    //Tempt
    // std::vector<cv::KeyPoint> vTempStatticKEys;
    // std::vector<cv::KeyPoint> vTempStatticKEysRight;
    cv::Mat newDescriptors;
    cv::Mat newDescriptorsRight;

    for( std::vector<cv::KeyPoint>::iterator itkp = mvKeys.begin(); itkp!=mvKeys.end();itkp++)
    {
        int nseg = (int)imSegLeft.at<uchar>(itkp->pt.y,itkp->pt.x);
        mvnSeg.push_back(nseg);
    }
    int kpidx = 0;//kp denotes the pointer to mvkeys, if erased, the pointer will auto point to next
    for (int segidx = 0;segidx < mvnSeg.size();segidx++){ // seg idx iterate all key points
      if (mvnSeg[segidx] == 0){
        kpidx++;

        newDescriptors.push_back(mDescriptors.row(segidx));
        // newDescriptorsRight.push_back(mDescriptorsRight.row(segidx));

      }
      else{
        // std::cerr << "erase something" << '\n';
        cv::KeyPoint kp = mvKeys[kpidx];
        // earse the dynamic points
        mvKeys.erase(mvKeys.begin() + kpidx);
        // mvKeysRight.erase(mvKeysRight.begin() + kpidx);

        //Append the dynamic points TODO associated with Humans;
        mvDyKeys.push_back(kp);
      }
    }

    mDescriptors.release();
    mDescriptors= newDescriptors.clone();
    // mDescriptorsRight.release();
    // mDescriptorsRight= newDescriptorsRight.clone();
    // std::cerr << "DEs" <<mDescriptorsRight.rows << mDescriptorsRight.cols << '\n';

    // mDescriptorsRight = newDescriptorsRight.clone();


}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

std::vector<cv::Mat> Frame::UnprojectStereoHuman(const int &i)
{
  human_pose mHuman = mvHumanPoses[i];
  std::vector<cv::Mat> v;
  for (int itr =0;itr<18;itr++){
    cv::Mat x3DHumanKey;
    const float z = mHuman.vHumanKeyDepths[itr];
    if(z>0)
    {
        const float u = mHuman.vHumanKeyPoints[itr].pt.x;
        const float v = mHuman.vHumanKeyPoints[itr].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        x3DHumanKey = mRwc*x3Dc+mOw;
    }
    else
    {
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << 0,0,0);
        x3DHumanKey = mRwc*x3Dc+mOw;
    }
    v.push_back(x3DHumanKey);
  }

  return v;
}

std::vector<bool> Frame::IsHumanInitBad(const int &i)
{
  human_pose mHuman = mvHumanPoses[i];
  std::vector<bool> v = mvHumanPoses[i].vbIsBad;
  return v;
}

} //namespace ORB_SLAM
