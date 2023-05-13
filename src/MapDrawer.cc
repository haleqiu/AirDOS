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
#include "MapDrawer.h"
#include "MapHumanPose.h"
#include "MapHumanTrajectory.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        if (mpMap->mbIsSeg)
        {
            int colorindex = mpMap->FindColor(vpMPs[i]->mnSeg);
            Eigen::Vector3f color = mpMap->mColorIdxMap[colorindex];
            glColor3f(color(0),color(1),color(2));
        }
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);//Temp hack in for visualizaiton

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        if (mpMap->mbIsSeg)
        {
            int colorindex = mpMap->FindColor((*sit)->mnSeg);
            Eigen::Vector3f color = mpMap->mColorIdxMap[colorindex];
            glColor3f(color(0),color(1),color(2));
        }
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {

            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();
            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawCurrentHumanPose(bool bfilter)
{
// Show the Human tracked currently by our system
  for (int id = 0; id<mpMap->mvCurrentTrackID.size(); id++){
    // DrawHumanPose(allHumanPoses[human_num]);
        int traj_id = mpMap->mvCurrentTrackID[id];
        if (traj_id <0) continue;

        MapHumanTrajectory* pMapHumanTrajectory = mpMap->GetMapHumanTrajectory(traj_id);
        if (!pMapHumanTrajectory)
            continue;  
        const std::vector<MapHumanPose*> &allHumanPoses = pMapHumanTrajectory->GetMapHumanTrajectory();
        int human_num = allHumanPoses.size() -1;
        // Plot the trajactory that has been optimized

        if (human_num > 4){ //if no trajactory to plot
            //plotting the trajactory based on the head position
            // starting from the first human pose to the next
            bool initflag = false;
            int first_human_id = 0;
            for (int human_id = 1; human_id<allHumanPoses.size(); human_id++)
            {
                bool isLost = allHumanPoses[human_id]->isLost;

                bool isOptimized1 = allHumanPoses[human_id]->isOptimized(1);

                cv::Mat pos2 = cv::Mat(1,1, CV_64F, double(0));
                cv::Mat pos1 = cv::Mat(1,1, CV_64F, double(0));

                int goodcount = 0;
                for (int body_part : mpMap->head)
                {
                    // bool isBad1 = allHumanPoses[human_id-1]->isBad(body_part);
                    bool isBad = allHumanPoses[human_id]->isBad(body_part);
                    bool isLost = allHumanPoses[human_id]->mvHumanKeyPos[body_part]->bIsLost;

                    // only count the good observation
                    // if ((isLost) || (isBad1 || isBad2))
                    if (isBad || isLost)
                        continue;
                    cv::Mat p2 = allHumanPoses[human_id]->GetHumanKeyPos(body_part);
                    cv::Mat p1 = allHumanPoses[first_human_id]->GetHumanKeyPos(body_part);

                    pos1 = pos1 + p1;
                    pos2 = pos2 + p2;
                    goodcount++;
                }
                // At least more than 2 good pose
                pos2 = pos2/goodcount;
                // pos1 = pos1/goodcount;
                if (goodcount < 1)
                    continue;

                if (!initflag){
                    first_human_id = human_id;
                    initflag=true;
                    continue;
                }

                int colorid = traj_id%mpMap->mColorIdxMap.size();
                Eigen::Vector3f colors = mpMap->mColorIdxMap.at(colorid);

                glLineWidth(2);
                glBegin(GL_LINES);
                glColor3f(colors[0], colors[1], colors[2]);
                glVertex3f(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2));
                glVertex3f(pos2.at<float>(0),pos2.at<float>(1),pos2.at<float>(2));
                glEnd();


                first_human_id = human_id;

                {
                DrawCircle(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2),0.1,10,colorid);
                DrawCircle(pos2.at<float>(0),pos2.at<float>(1),pos2.at<float>(2),0.1,10,colorid);
                }

                // Draw every pose if want the all humantraj
                if (human_id%3==0)
                    DrawHumanPoseColor(allHumanPoses[human_id],colorid );
            }
        }
  }
}

// A small utils to show the trajectory of humans
void MapDrawer::DrawCircle(float cx, float y, float cz, float r, int num_segments, int colorid) {
    Eigen::Vector3f colors = mpMap->mColorIdxMap.at(colorid);
    glColor3f(colors[0], colors[1], colors[2]);

    glBegin(GL_LINE_LOOP);
    for (int ii = 0; ii < num_segments; ii++)   {
        float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle
        float x = r * cosf(theta);//calculate the x component
        float z = r * sinf(theta);//calculate the y component
        glVertex3f(x + cx, y, z + cz);//output vertex
    }
    glEnd();
}

// Draw the current trajectory and the optimized trajectory.
void MapDrawer::DrawAllHumanTrajactory(bool bfilter, bool bOptimizedTrajactory){
    // Merge the current tracked trajectory
    std::set<int> setTrajIdtoView;
    for (int x : mpMap->mvCurrentTrackID) setTrajIdtoView.insert(x);
    for (int x : mpMap->msetOptimizedTrackID){setTrajIdtoView.insert(x);}
    if (mpMap->mbIsStaticOnly){
        for (std::pair<int, MapHumanTrajectory*> x : mpMap->mmapHumanTrajectory){setTrajIdtoView.insert(x.first);}
    }
    // Iterate the concated vector
    for (int traj_id : setTrajIdtoView){
        if (traj_id <0) continue;

        MapHumanTrajectory* pMapHumanTrajectory = mpMap->GetMapHumanTrajectory(traj_id);
        if (!pMapHumanTrajectory)
        continue;
        const std::vector<MapHumanPose*> &allHumanPoses = pMapHumanTrajectory->GetMapHumanTrajectory();
        int human_num = allHumanPoses.size() -1;
        // Plot the trajactory that has been optimized

        if (human_num > 6){ //if no trajactory to plot
            //plotting the trajactory based on the head position
            // starting from the first human pose to the next
            bool initflag = false;
            int first_human_id = 0;
            for (int human_id = 0; human_id<allHumanPoses.size(); human_id++)
            {
                bool isLost = allHumanPoses[human_id]->isLost;

                bool isOptimized1 = allHumanPoses[human_id]->isOptimized(1);

                cv::Mat pos2 = cv::Mat(1,1, CV_64F, double(0));
                cv::Mat pos1 = cv::Mat(1,1, CV_64F, double(0));

                // if(!(human_id%3==0)){

                // TODO
                int goodcount = 0;
                for (int body_part : mpMap->head)
                {
                    // TODO the lost flag should also been sealed
                    // bool isBad1 = allHumanPoses[human_id-1]->isBad(body_part);
                    bool isBad = allHumanPoses[human_id]->isBad(body_part);
                    bool isLost = allHumanPoses[human_id]->mvHumanKeyPos[body_part]->bIsLost;

                    // only count the good observation
                    // if ((isLost) || (isBad1 || isBad2))
                    if (isBad || isLost)
                        continue;
                    cv::Mat p2 = allHumanPoses[human_id]->GetHumanKeyPos(body_part);
                    cv::Mat p1 = allHumanPoses[first_human_id]->GetHumanKeyPos(body_part);
                    if (body_part==1){
                    pos1 = pos1 + p1;
                    pos2 = pos2 + p2;
                    }
                    goodcount++;
                }
                // At least more than 2 good pose
                //pos2 = pos2/goodcount;
                // pos1 = pos1/goodcount;
                if (goodcount < 1)
                    continue;

                if (!initflag){
                    first_human_id = human_id;
                    initflag=true;
                    continue;
                }

                int colorid = traj_id%mpMap->mColorIdxMap.size();
                Eigen::Vector3f colors = mpMap->mColorIdxMap.at(colorid);
                //if (isOptimized1 || isOptimized2)
                 //   colors = Eigen::Vector3f(1,0,0);

                glLineWidth(2);
                glBegin(GL_LINES);
                glColor3f(colors[0], colors[1], colors[2]);
                glVertex3f(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2));
                glVertex3f(pos2.at<float>(0),pos2.at<float>(1),pos2.at<float>(2));
                glEnd();


                first_human_id = human_id;



                if (bOptimizedTrajactory)
                {
                DrawCircle(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2),0.1,10,colorid);
                DrawCircle(pos2.at<float>(0),pos2.at<float>(1),pos2.at<float>(2),0.1,10,colorid);
                }

                // Draw every pose if want the all humantraj
                if ((!bOptimizedTrajactory))
                    DrawHumanPoseColor(allHumanPoses[human_id],colorid );
            }
        }
    }
}

void MapDrawer::DrawAllHumanPose(bool bfilter){

    const vector<MapHumanPose*> &allHumanPoses  = mpMap->GetAllMapHumanPoses();

    for (int human_id = 0; human_id<allHumanPoses.size(); human_id++){
        DrawHumanPose(allHumanPoses[human_id]);
    }
}


//TODO this is not finished
void MapDrawer::DrawMotion(MapHumanPose* mHumanPose, cv::Mat Motion){
  glLineWidth(2);
  glColor3f(1.0f,0.0f,0.0f);
  glBegin(GL_LINES);
    cv::Mat MatCurrentPosition = mHumanPose->GetHumanKeyPos(0);
    // Map to non-corrected camera
    cv::Mat Rcw = Motion.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Motion.rowRange(0,3).col(3);
    cv::Mat PredictedPosition = Rcw*MatCurrentPosition+tcw;

    Eigen::Vector3f colors = Eigen::Vector3f(1,0,0);
    glColor3f(colors[0], colors[1], colors[2]);

    glVertex3f(MatCurrentPosition.at<float>(0),MatCurrentPosition.at<float>(1),MatCurrentPosition.at<float>(2)+0.5);
    glVertex3f(PredictedPosition.at<float>(0),PredictedPosition.at<float>(1),PredictedPosition.at<float>(2)+0.5);
    glEnd();
}

void MapDrawer::DrawHumanPose(MapHumanPose* mHumanPose){
  glLineWidth(2);
  glColor3f(1.0f,0.0f,0.0f);
  glBegin(GL_LINES);
  for(int itr = 0; itr < mHumanPose->mvHumanKeyPair.size(); itr++){
    int id1 = mpMap->body1[itr]; int id2 = mpMap->body2[itr];
    // shall we also check the point?
    //if (mHumanPose->mvHumanKeyPair[itr].bIsBad) continue;
    //if (mHumanPose->isBad(id1) || mHumanPose->isBad(id2)) continue;

    cv::Mat pos1 = mHumanPose->GetHumanKeyPos(id1);
    cv::Mat pos2 = mHumanPose->GetHumanKeyPos(id2);

    Eigen::Vector3f colors = mpMap->colormap.at(itr);//TODO not large than 10, hahahaha
    //
    // if (isOptimized1 || isOptimized2){
    //     colors = Eigen::Vector3f(1,0,0);
    // }
    glColor3f(colors[0], colors[1], colors[2]);

    glVertex3f(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2));
    glVertex3f(pos2.at<float>(0),pos2.at<float>(1),pos2.at<float>(2));
  }
  glEnd();
}

void MapDrawer::DrawHumanPoseColor(MapHumanPose* mHumanPose, int colorid){
  glLineWidth(3);
  glBegin(GL_LINES);
// glBegin(GL_POINTS);
//   glPointSize(5);
    Eigen::Vector3f colors = mpMap->mColorIdxMap.at(colorid);;//TODO not large than 10, hahahaha
  glColor3f(colors[0], colors[1], colors[2]);
  for(int itr = 0; itr < mHumanPose->mvHumanKeyPair.size(); itr++){
    int id1 = mpMap->body1[itr]; int id2 = mpMap->body2[itr];
    // shall we also check the point?
    //if (mHumanPose->mvHumanKeyPair[itr].bIsBad) continue;
    //if (mHumanPose->isBad(id1) || mHumanPose->isBad(id2)) continue;

    cv::Mat pos1 = mHumanPose->GetHumanKeyPos(id1);
    cv::Mat pos2 = mHumanPose->GetHumanKeyPos(id2);

    glVertex3f(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2));
    glVertex3f(pos2.at<float>(0),pos2.at<float>(1),pos2.at<float>(2));
  }
  glEnd();
}

void MapDrawer::DrawCameraGT(std::vector<std::pair<int, cv::Mat>> vGTPose){
    int lens = vGTPose.size();
    glLineWidth(2);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < lens; i++){
        glVertex3f(vGTPose[i].second.at<float>(0,3), vGTPose[i].second.at<float>(1,3) , vGTPose[i].second.at<float>(2,3));
    }
    glEnd();
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
