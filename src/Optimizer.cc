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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include <fstream>
#include <iostream>
#include <string>

#include "Converter.h"
#include "MapHumanPose.h"
#include "MapHumanTrajectory.h"
#include "g2o_vertex_distance.h"
#include "g2o_edge_rigidbody.h"
#include "g2o_dyn_slam3d.h"
//This is a tempt solution
#include "g2o_vertex_se3.h"
#include <opencv2/core/eigen.hpp>

#include<mutex>

namespace ORB_SLAM2 {


    void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                           const bool bRobust) {
      vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
      vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
      BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
    }


    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                     const bool bRobust) {
      vector<bool> vbNotIncludedMP;
      vbNotIncludedMP.resize(vpMP.size());

      g2o::SparseOptimizer optimizer;
      g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

      linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

      g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

      long unsigned int maxKFid = 0;

      // Set KeyFrame vertices
      for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
          continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKF->mnId > maxKFid)
          maxKFid = pKF->mnId;
      }

      const float thHuber2D = sqrt(5.99);
      const float thHuber3D = sqrt(7.815);

      // Set MapPoint vertices
      for (size_t i = 0; i < vpMP.size(); i++) {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
          continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

          KeyFrame *pKF = mit->first;
          if (pKF->isBad() || pKF->mnId > maxKFid)
            continue;

          nEdges++;

          const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

          if (pKF->mvuRight[mit->second] < 0) {
            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
            e->setMeasurement(obs);
            const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            if (bRobust) {
              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuber2D);
            }

            e->fx = pKF->fx;
            e->fy = pKF->fy;
            e->cx = pKF->cx;
            e->cy = pKF->cy;

            optimizer.addEdge(e);
          } else {
            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKF->mvuRight[mit->second];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
            e->setMeasurement(obs);
            const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            if (bRobust) {
              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuber3D);
            }

            e->fx = pKF->fx;
            e->fy = pKF->fy;
            e->cx = pKF->cx;
            e->cy = pKF->cy;
            e->bf = pKF->mbf;

            optimizer.addEdge(e);
          }
        }

        if (nEdges == 0) {
          optimizer.removeVertex(vPoint);
          vbNotIncludedMP[i] = true;
        } else {
          vbNotIncludedMP[i] = false;
        }
      }

      // Optimize!
      optimizer.initializeOptimization();
      optimizer.optimize(nIterations);

      // Recover optimized data

      //Keyframes
      for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
          continue;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (nLoopKF == 0) {
          pKF->SetPose(Converter::toCvMat(SE3quat));
        } else {
          pKF->mTcwGBA.create(4, 4, CV_32F);
          Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
          pKF->mnBAGlobalForKF = nLoopKF;
        }
      }

      //Points
      for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
          continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
          continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
          pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
          pMP->UpdateNormalAndDepth();
        } else {
          pMP->mPosGBA.create(3, 1, CV_32F);
          Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
          pMP->mnBAGlobalForKF = nLoopKF;
        }
      }

    }

    int Optimizer::PoseOptimization(Frame *pFrame) {
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

      g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      int nInitialCorrespondences = 0;

      // Set Frame vertex
      g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
      vSE3->setId(0);
      vSE3->setFixed(false);
      optimizer.addVertex(vSE3);

      // Set MapPoint vertices
      const int N = pFrame->N;

      vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
      vector<size_t> vnIndexEdgeMono;
      vpEdgesMono.reserve(N);
      vnIndexEdgeMono.reserve(N);

      vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
      vector<size_t> vnIndexEdgeStereo;
      vpEdgesStereo.reserve(N);
      vnIndexEdgeStereo.reserve(N);

      const float deltaMono = sqrt(5.991);
      const float deltaStereo = sqrt(7.815);


      {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++) {
          MapPoint *pMP = pFrame->mvpMapPoints[i];
          if (pMP) {
            // Monocular observation
            if (pFrame->mvuRight[i] < 0) {
              nInitialCorrespondences++;
              pFrame->mvbOutlier[i] = false;

              Eigen::Matrix<double, 2, 1> obs;
              const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
              obs << kpUn.pt.x, kpUn.pt.y;

              g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
              e->setMeasurement(obs);
              const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
              e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(deltaMono);

              e->fx = pFrame->fx;
              e->fy = pFrame->fy;
              e->cx = pFrame->cx;
              e->cy = pFrame->cy;
              cv::Mat Xw = pMP->GetWorldPos();
              e->Xw[0] = Xw.at<float>(0);
              e->Xw[1] = Xw.at<float>(1);
              e->Xw[2] = Xw.at<float>(2);

              optimizer.addEdge(e);

              vpEdgesMono.push_back(e);
              vnIndexEdgeMono.push_back(i);
            } else  // Stereo observation
            {
              nInitialCorrespondences++;
              pFrame->mvbOutlier[i] = false;

              //SET EDGE
              Eigen::Matrix<double, 3, 1> obs;
              const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
              const float &kp_ur = pFrame->mvuRight[i];
              obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

              g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
              e->setMeasurement(obs);
              const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
              Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
              e->setInformation(Info);

              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(deltaStereo);

              e->fx = pFrame->fx;
              e->fy = pFrame->fy;
              e->cx = pFrame->cx;
              e->cy = pFrame->cy;
              e->bf = pFrame->mbf;
              cv::Mat Xw = pMP->GetWorldPos();
              e->Xw[0] = Xw.at<float>(0);
              e->Xw[1] = Xw.at<float>(1);
              e->Xw[2] = Xw.at<float>(2);

              optimizer.addEdge(e);

              vpEdgesStereo.push_back(e);
              vnIndexEdgeStereo.push_back(i);
            }
          }

        }
      }


      if (nInitialCorrespondences < 3)
        return 0;

      // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
      // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
      const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
      const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
      const int its[4] = {10, 10, 10, 10};

      int nBad = 0;
      // 4 iterations of optimization
      for (size_t it = 0; it < 4; it++) {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
          g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

          const size_t idx = vnIndexEdgeMono[i];

          if (pFrame->mvbOutlier[idx]) {
            e->computeError();
          }

          const float chi2 = e->chi2();

          if (chi2 > chi2Mono[it]) {
            pFrame->mvbOutlier[idx] = true;
            e->setLevel(1);
            nBad++;
          } else {
            pFrame->mvbOutlier[idx] = false;
            e->setLevel(0);
          }

          if (it == 2)
            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
          g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

          const size_t idx = vnIndexEdgeStereo[i];

          if (pFrame->mvbOutlier[idx]) {
            e->computeError();
          }

          const float chi2 = e->chi2();

          if (chi2 > chi2Stereo[it]) {
            pFrame->mvbOutlier[idx] = true;
            e->setLevel(1);
            nBad++;
          } else {
            e->setLevel(0);
            pFrame->mvbOutlier[idx] = false;
          }

          if (it == 2)
            e->setRobustKernel(0);
        }

        if (optimizer.edges().size() < 10)
          break;
      }

      // Recover optimized pose and return number of inliers
      g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
      g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
      cv::Mat pose = Converter::toCvMat(SE3quat_recov);
      pFrame->SetPose(pose);

      return nInitialCorrespondences - nBad;
    }

    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap) {
      // Local KeyFrames: First Breath Search from Current Keyframe
      list<KeyFrame *> lLocalKeyFrames;

      lLocalKeyFrames.push_back(pKF);
      pKF->mnBALocalForKF = pKF->mnId;

      const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
      for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad())
          lLocalKeyFrames.push_back(pKFi);
      }

      // Local MapPoints seen in Local KeyFrames
      list<MapPoint *> lLocalMapPoints;
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
          MapPoint *pMP = *vit;
          if (pMP)
            if (!pMP->isBad())
              if (pMP->mnBALocalForKF != pKF->mnId) {
                lLocalMapPoints.push_back(pMP);
                pMP->mnBALocalForKF = pKF->mnId;
              }
        }
      }

      // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
      list<KeyFrame *> lFixedCameras;
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
          KeyFrame *pKFi = mit->first;

          if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
            pKFi->mnBAFixedForKF = pKF->mnId;
            if (!pKFi->isBad())
              lFixedCameras.push_back(pKFi);
          }
        }
      }

      // Setup optimizer
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

      linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

      g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

      unsigned long maxKFid = 0;

      // Set Local KeyFrame vertices
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
          maxKFid = pKFi->mnId;
      }

      // Set Fixed KeyFrame vertices
      for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
          maxKFid = pKFi->mnId;
      }

      // Set MapPoint vertices
      const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

      vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
      vpEdgesMono.reserve(nExpectedSize);

      vector<KeyFrame *> vpEdgeKFMono;
      vpEdgeKFMono.reserve(nExpectedSize);

      vector<MapPoint *> vpMapPointEdgeMono;
      vpMapPointEdgeMono.reserve(nExpectedSize);

      vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
      vpEdgesStereo.reserve(nExpectedSize);

      vector<KeyFrame *> vpEdgeKFStereo;
      vpEdgeKFStereo.reserve(nExpectedSize);

      vector<MapPoint *> vpMapPointEdgeStereo;
      vpMapPointEdgeStereo.reserve(nExpectedSize);

      const float thHuberMono = sqrt(5.991);
      const float thHuberStereo = sqrt(7.815);

      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
          KeyFrame *pKFi = mit->first;

          if (!pKFi->isBad()) {
            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

            // Monocular observation
            if (pKFi->mvuRight[mit->second] < 0) {
              Eigen::Matrix<double, 2, 1> obs;
              obs << kpUn.pt.x, kpUn.pt.y;

              g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
              e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
              e->setMeasurement(obs);
              const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
              e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuberMono);

              e->fx = pKFi->fx;
              e->fy = pKFi->fy;
              e->cx = pKFi->cx;
              e->cy = pKFi->cy;

              optimizer.addEdge(e);
              vpEdgesMono.push_back(e);
              vpEdgeKFMono.push_back(pKFi);
              vpMapPointEdgeMono.push_back(pMP);
            } else // Stereo observation
            {
              Eigen::Matrix<double, 3, 1> obs;
              const float kp_ur = pKFi->mvuRight[mit->second];
              obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

              g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
              e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
              e->setMeasurement(obs);
              const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
              Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
              e->setInformation(Info);

              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuberStereo);

              e->fx = pKFi->fx;
              e->fy = pKFi->fy;
              e->cx = pKFi->cx;
              e->cy = pKFi->cy;
              e->bf = pKFi->mbf;

              optimizer.addEdge(e);
              vpEdgesStereo.push_back(e);
              vpEdgeKFStereo.push_back(pKFi);
              vpMapPointEdgeStereo.push_back(pMP);
            }
          }
        }
      }

      if (pbStopFlag)
        if (*pbStopFlag)
          return;

      optimizer.initializeOptimization();
      optimizer.optimize(5);

      bool bDoMore = true;

      if (pbStopFlag)
        if (*pbStopFlag)
          bDoMore = false;

      if (bDoMore) {

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
          g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
          MapPoint *pMP = vpMapPointEdgeMono[i];

          if (pMP->isBad())
            continue;

          if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
          g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
          MapPoint *pMP = vpMapPointEdgeStereo[i];

          if (pMP->isBad())
            continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

      }

      vector<pair<KeyFrame *, MapPoint *> > vToErase;
      vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

      // Check inlier observations
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
          continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
          KeyFrame *pKFi = vpEdgeKFMono[i];
          vToErase.push_back(make_pair(pKFi, pMP));
        }
      }

      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
          continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          KeyFrame *pKFi = vpEdgeKFStereo[i];
          vToErase.push_back(make_pair(pKFi, pMP));
        }
      }

      // Get Map Mutex
      unique_lock<mutex> lock(pMap->mMutexMapUpdate);

      if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
          KeyFrame *pKFi = vToErase[i].first;
          MapPoint *pMPi = vToErase[i].second;
          pKFi->EraseMapPointMatch(pMPi);
          pMPi->EraseObservation(pKFi);
        }
      }

      // Recover optimized data

      //Keyframes
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
      }

      //Points
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
      }
    }

// This method is adding the dynamic object we observed into consideration
// I think I can try update the initialized position of the human pose
// But it seems meaningless, cauze there is no difference.
    void Optimizer::LocalBundleAdjustmentHumanTrajactoryFast(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                                             float SigmaStatic, float SigmaHuman, float SigmaRigidity,
                                                             float SigmaMotion, float thRanSacMotion,
                                                             float thRanSacRigidity) {
      //Here the local bundle adjustment taking from human trajectory
      bool bRobust = true;
      const float thHuberStereo = sqrt(7.815);
      const float thHuberMotion = sqrt(thRanSacMotion);
      const float thHuberRigidity = sqrt(thRanSacRigidity);

      list<KeyFrame *> lLocalKeyFrames;
      lLocalKeyFrames.push_back(pKF);
      pKF->mnBALocalForHM = pKF->mnId;

      // Enqueue Local KF based on Covisibility graph
      const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
      for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForHM = pKF->mnId;
        if (!pKFi->isBad())
          lLocalKeyFrames.push_back(pKFi);
      }
      std::cerr << "thRanSacMotion: " << thRanSacMotion << "SigmaMotion: " << SigmaMotion << '\n';

      vector<MapHumanPose *> vpMapHumanPoses = pKF->mvpMapHumanPoses;
      list<MapHumanTrajectory *> lLocalMapHumanTrajactories;
      list<MapHumanPose *> lLocalMapHumanPose;
      // Now input every huamn trajactory regardless the possitbility of a bad traj
      for (vector<MapHumanPose *>::iterator hmit = vpMapHumanPoses.begin(); hmit != vpMapHumanPoses.end(); hmit++) {
        if (*hmit) {
          int nTrackID = (*hmit)->mnTrackId;
          // find the trajactory through id from mmaplocal
          MapHumanTrajectory *pMapHumanTrajectory = pMap->GetMapHumanTrajectory(nTrackID);
          if (!pMapHumanTrajectory) continue;
          // Set the local BA id for the later fixed camera
          pMapHumanTrajectory->mnBALocalForHM = pKF->mnId;
          // Check the length of the MapHumanTrajectory
          if (pMapHumanTrajectory->mnHumanPoses > pMap->thLongTrajectory) {
            // if (pMapHumanTrajectory->mnBadTrack/(pMapHumanTrajectory->mnHumanPoses*5) < 0.2)
            lLocalMapHumanTrajactories.push_back(pMapHumanTrajectory);
          }
        }
      }

      // Enqueue the Local Key Frame which observes local localtrajactory
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        const std::map<KeyFrame *, size_t> mapKFObserved = (*htit)->GetObservations();
        for (std::map<KeyFrame *, size_t>::const_iterator itkf = mapKFObserved.begin();
             itkf != mapKFObserved.end(); itkf++) {
          KeyFrame *pKFi = itkf->first;
          // insert the KF that have seen current human pose. // Currently there will have no skip situation
          if (pKFi->mnBALocalForHM != pKF->mnId) {
            // here record the HMT that we are going to run BA
            pKFi->mnBALocalForHM = pKF->mnId;
            // TODO what if the HMpose observed have some view of the bad kf?????
            if (!pKFi->isBad())
              lLocalKeyFrames.push_back(pKFi);

          }
        }
      }

      // Enqueue the MapHumanPose from the Trajactory

      // Local MapPoints seen in Local KeyFrames
      // Set the variables of mnBALocalForHM for the local map points
      // Later enque the lFixedCamera based on observations
      list<MapPoint *> lLocalMapPoints;
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
          MapPoint *pMP = *vit;
          if (pMP)
            if (!pMP->isBad())
              if (pMP->mnBALocalForHM != pKF->mnId) {
                lLocalMapPoints.push_back(pMP);
                pMP->mnBALocalForHM = pKF->mnId;
              }
        }
      }
      list<KeyFrame *> lFixedCameras;
      // Fixed camera from local map points
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
          KeyFrame *pKFi = mit->first;

          if (pKFi->mnBALocalForHM != pKF->mnId && pKFi->mnBAFixedForHM != pKF->mnId) {
            pKFi->mnBAFixedForHM = pKF->mnId;

            if (!pKFi->isBad())
              lFixedCameras.push_back(pKFi);
          }
        }
      }


      // The total number of stereo observation, not include the HumanTrajactory, not include the lFixedCameras that comming from lFixedLocalMapHumanTrajactories
      const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();
      // This is the group of stereo observation
      vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
      vpEdgesStereo.reserve(nExpectedSize);

      vector<KeyFrame *> vpEdgeKFStereo;
      vpEdgeKFStereo.reserve(nExpectedSize);

      vector<MapPoint *> vpMapPointEdgeStereo;
      vpMapPointEdgeStereo.reserve(nExpectedSize);

      // The group of Human Key points observation
      // Save the pointer of HumanKeys seems easier, but I might need more operation.
      vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgeHumanKeys;
      vector<std::pair<MapHumanPose *, int>> vpObservationHumanKeys;

      // Here to save the vertex and edge for rigidity constraint; one corresponding to one
      // A simple solution will be saving the keypair and the MapHumanPose
      vector<EdgeRigidBodyDouble *> vpEdgeRigidity;
      vector<std::pair<MapHumanPose *, int>> vpSegmentRigidity;

      // The edge for motion constraint and the
      vector<LandmarkMotionTernaryEdge *> vpEdgeMotion;
      vector<std::pair<MapHumanPose *, int>> vpFirstHumanPose;
      vector<std::pair<MapHumanPose *, int>> vpSecondHumanPose;

      // Finding the Trajactory inside, which may not be seen in current frame (will that be too much?)
      list<MapHumanTrajectory *> lFixedLocalMapHumanTrajactories;
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        vector<MapHumanPose *> vpMapHumanPoses = (*lit)->mvpMapHumanPoses;
        for (vector<MapHumanPose *>::iterator itmp = vpMapHumanPoses.begin(); itmp != vpMapHumanPoses.end(); itmp++) {
          if (*itmp) {
            int nTrackID = (*itmp)->mnTrackId;
            // find the trajactory through id from mmaplocal
            MapHumanTrajectory *pMapHumanTrajectory = pMap->GetMapHumanTrajectory(nTrackID);
            if (!pMapHumanTrajectory) continue;
            // if not a local camera, then a fixed
            if (pMapHumanTrajectory->mnBALocalForHM != pKF->mnId && pMapHumanTrajectory->mnBAFixedForHM != pKF->mnId) {
              pMapHumanTrajectory->mnBAFixedForHM = pKF->mnId;
              if (pMapHumanTrajectory->mnHumanPoses > pMap->thLongTrajectory)
                lFixedLocalMapHumanTrajactories.push_back(pMapHumanTrajectory);
            }
          }
        }
      }

      // Enqueue the Fixed camera that have seen those human trajectory
      for (list<MapHumanTrajectory *>::iterator hmtit = lFixedLocalMapHumanTrajactories.begin(),
               lend = lFixedLocalMapHumanTrajactories.end(); hmtit != lend; hmtit++) {
        const std::map<KeyFrame *, size_t> mapObservations = (*hmtit)->GetObservations();
        for (std::map<KeyFrame *, size_t>::const_iterator itobs = mapObservations.begin();
             itobs != mapObservations.end(); itobs++) {
          KeyFrame *pKFi = itobs->first;
          if (pKFi->mnBALocalForHM != pKF->mnId && pKFi->mnBAFixedForHM != pKF->mnId) {
            pKFi->mnBAFixedForHM = pKF->mnId;
            if (!pKFi->isBad())
              lFixedCameras.push_back(pKFi);
          }
        }
      }

      // DEBUG
      std::cerr << "the length of lLocalKeyFrames" << lLocalKeyFrames.size() << '\n';
      std::cerr << "the length of lFixedCameras" << lFixedCameras.size() << '\n';
      std::cerr << "the length of lLocalMapPoints" << lLocalMapPoints.size() << '\n';
      std::cerr << "the length of lFixedLocalMapHumanTrajactories " << lFixedLocalMapHumanTrajactories.size() << '\n';
      std::cerr << "the length of lLocalMapHumanTrajactories " << lLocalMapHumanTrajactories.size() << '\n';
      // TODO how about the size of the rigidity and the human pose

      // Merge the FixedHMT to optimized the whole trajectory
      lLocalMapHumanTrajactories.merge(lFixedLocalMapHumanTrajactories);

      //set up optimizer
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolverX::LinearSolverType *linearSolver; // BlockSolverX instead of BlockSolver63
      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
      g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      long unsigned maxKFid = 0;
      long unsigned maxMPid = 0;
      long unsigned maxRGid = 0;
      long unsigned maxHPid = 0;
      long unsigned maxMTid = 0;


      // Set the KeyFrame in the lLocalKeyFrames
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
          maxKFid = pKFi->mnId;
      }

      // Set Fixed KeyFrame Verteices
      for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
          maxKFid = pKFi->mnId;
      }

      std::cerr << "the maxKFid " << maxKFid << '\n';

      // Enqueue the observation of the local map points.
      // Set the edges for observation, This should be the same as the previous setup
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        if (id > maxMPid)
          maxMPid = id;

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();
        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
          KeyFrame *pKFi = mit->first;

          if (!pKFi->isBad()) {
            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKFi->mvuRight[mit->second];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);

            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;
            e->bf = pKFi->mbf;

            // Save the vertex of KF, the map points and edges.
            // This is the same as previous KeyFrame
            optimizer.addEdge(e);
            vpEdgesStereo.push_back(e);
            vpEdgeKFStereo.push_back(pKFi);
            vpMapPointEdgeStereo.push_back(pMP);
          }
        }
      }
      std::cerr << "max MP " << maxMPid << '\n';

      // set vertex rigidty in the local map human trajactories,
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        MapHumanTrajectory *pHMT = (*htit);
        std::vector<Rigidbody> vRigidity = (*htit)->mvRigidBodys;
        // read the initial set up of the rigidty segments // TODO enqueue to v
        for (std::vector<Rigidbody>::const_iterator itRigid = vRigidity.begin();
             itRigid != vRigidity.end(); itRigid++) {
          VertexDistanceDouble *vRigid = new VertexDistanceDouble();
          int id = maxMPid + itRigid->mnId + 1;
          vRigid->setId(id);

          // if the segemnt is optimized, I would like to take it for initialization
          if (itRigid->isBad && !itRigid->isOptimized) {
            vRigid->setEstimate(0);
          } else {
            vRigid->setEstimate(itRigid->mnDistance);
          }
          optimizer.addVertex(vRigid);

          if (id > maxRGid)
            maxRGid = id;
        }
      }
      std::cerr << "maxRGid " << maxRGid << '\n';
      // TODO lFixed Trajactory
      // Set the vertex of the motion
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        int id = (*htit)->mnId + maxRGid + 1;
        if (id > maxMTid)
          maxMTid = id;
        VertexSE3 *vSE3 = new VertexSE3();
        Isometry3 estimation = Isometry3::Identity();
        if ((*htit)->isOptimized)
          Isometry3 estimation(Converter::toEigenMatrix((*htit)->mTMotion));
        vSE3->setEstimate(estimation);
        vSE3->setId(id);
        optimizer.addVertex(vSE3);
      }
      std::cerr << "maxMTid " << maxMTid << '\n';


      // Set the vertex Human Pose and the edges KeyFrame
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        vector<MapHumanPose *> vHumanPoses = (*htit)->GetMapHumanTrajectory();

        // Adding vertex for HumanPose, edges for human key point observation and edges for rigidity
        for (vector<MapHumanPose *>::iterator hmit = vHumanPoses.begin(); hmit != vHumanPoses.end(); hmit++) {
          MapHumanPose *pMapHumanpose = *hmit;
          if (pMapHumanpose->isEarsed)
            continue;

          // HumanKey will not be excluded if key isBad in the initializaion. Because I want them to be fixed
          // during the fixed inlier optimization
          // The mnbodyparts is 14, iterate all the possible key points on human
          for (int itr = 0; itr < pMap->mnbodyparts; itr++) {
            // dealling with the humanpose id
            MapHumanKey *pMapKey = pMapHumanpose->mvHumanKeyPos[itr];
            int id = pMapKey->mnId + maxMTid + 1;
            if (id > maxHPid) maxHPid = id;

            // setting up the vertex for human key
            g2o::VertexSBAPointXYZ *vHumanKey = new g2o::VertexSBAPointXYZ();
            vHumanKey->setEstimate(Converter::toVector3d(pMapKey->WorldPos));
            if (pMapKey->WorldPos.empty()) std::cerr << "empty id" << id << '\n';
            vHumanKey->setId(id);
            optimizer.addVertex(vHumanKey);

            // if the key is bad, I will not take this observation into edges,
            // Leave it for the rigidity and the motion constraint
            if (pMapKey->bIsBad)
              continue;

            // if the human pose is observed by the keyframe, we can constraint the obs
            if (pMapHumanpose->mbIsInKeyFrame) {
              KeyFrame *pKF = (pMapHumanpose->mObservations).first;
              int obsid = pMapHumanpose->mObservations.second;//the mapping for corresponding position
              if (pKF->isBad()) continue;

              Eigen::Matrix<double, 3, 1> obs;
              human_pose mHuman = pKF->mvHumanPoses[obsid];
              const cv::KeyPoint kpUn = mHuman.vHumanKeyPoints[itr];
              const float kp_ur = mHuman.vHumanKeyPointsRight[itr].pt.x;
              obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

              g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id));
              if (vPoint == nullptr) {
                cerr << " nullptr hm point" << std::endl;
                continue;
              }

              // Add project constraint to hmp
              g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();
              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
              e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
              e->setMeasurement(obs);

              // TODO the information matrix, can be initiate with the probability from human poses
              const float &invSigma2 = SigmaHuman;
              Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
              e->setInformation(Info);

              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuberStereo);

              e->fx = pKF->fx;
              e->fy = pKF->fy;
              e->cx = pKF->cx;
              e->cy = pKF->cy;
              e->bf = pKF->mbf;

              optimizer.addEdge(e);

              // Record the edges, prepare for the ransacs
              vpEdgeHumanKeys.push_back(e);
              vpObservationHumanKeys.push_back(std::make_pair(pMapHumanpose, obsid));
            }
          }

          // Adding rigidity constraint
          // Doing this with idx since we need to save the order in
          for (int i = 0; i < pMapHumanpose->mvHumanKeyPair.size(); i++) {

            HumanKeyPair pairHumanKeys = pMapHumanpose->mvHumanKeyPair[i];

            // TODO can the observation of human key modify this ?
            // if (pairHumanKeys.isBad) continue;

            int id1 = pairHumanKeys.idFirstKey + maxMTid + 1;
            int id2 = pairHumanKeys.idSecondKey + maxMTid + 1;
            int id3 = pairHumanKeys.idDistance + maxMPid + 1; // May have problem

            g2o::VertexSBAPointXYZ *vHumanKey1 = dynamic_cast<g2o::VertexSBAPointXYZ *>( optimizer.vertex(id1));
            g2o::VertexSBAPointXYZ *vHumanKey2 = dynamic_cast<g2o::VertexSBAPointXYZ *>( optimizer.vertex(id2));
            EdgeRigidBodyDouble *edgeRigid = new EdgeRigidBodyDouble();

            if ((vHumanKey1 == nullptr) || (vHumanKey2 == nullptr)) {
              continue;
            }

            edgeRigid->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id1)));
            edgeRigid->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id2)));
            edgeRigid->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id3)));

            Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(1, 1);
            Info(0, 0) = SigmaRigidity;
            edgeRigid->setInformation(Info);

            // TODO find the appropriate huber
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            edgeRigid->setRobustKernel(rk);
            rk->setDelta(thRanSacRigidity);

            optimizer.addEdge(edgeRigid);
            vpEdgeRigidity.push_back(edgeRigid);
            // std::pair<*MapHumanPose,int> pairRigidSegment (pMapHumanpose, i);
            vpSegmentRigidity.push_back(std::make_pair(pMapHumanpose, i));
          }
        }

        // Motion constraint
        int motionid = (*htit)->mnId + maxRGid + 1;
        vector<MapHumanPose *>::iterator firsthmit = vHumanPoses.begin();
        vector<MapHumanPose *>::iterator sechmit = std::next(firsthmit);
        // (*firsthmit)->isLost=true;
        while (sechmit != vHumanPoses.end()) {

          double time_range = (*sechmit)->mTimeStamp - (*firsthmit)->mTimeStamp;

          for (int i = 0; i < pMap->mimainskleton; i++) {
            int itr = pMap->mainskleton[i];

            MapHumanKey *pfisrtMapKey = (*firsthmit)->mvHumanKeyPos[itr];
            int id1 = pfisrtMapKey->mnId + maxMTid + 1;
            MapHumanKey *pMapKey = (*sechmit)->mvHumanKeyPos[itr];
            int id2 = pMapKey->mnId + maxMTid + 1;

            // bool isLost = pfisrtMapKey->bIsLost;
            bool isLost = false;

            LandmarkMotionTernaryEdge *em = new LandmarkMotionTernaryEdge();

            g2o::VertexSBAPointXYZ *vPoint1 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id1));
            g2o::VertexSBAPointXYZ *vPoint2 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id2));
            VertexSE3 *vSE3 = static_cast<VertexSE3 *>(optimizer.vertex(motionid));

            if ((vSE3 == nullptr) || (vPoint1 == nullptr) || (vPoint2 == nullptr)) {
              // TODO
              std::cerr << "nullptr" << "\n";
              continue;
            }

            em->setVertex(0, optimizer.vertex(id1));
            em->setVertex(1, optimizer.vertex(id2));
            em->setVertex(2, optimizer.vertex(motionid));
            em->delta_t = time_range;
            em->setMeasurement(Eigen::Vector3d(0, 0, 0));

            // TODO consider the huber function
            if (bRobust) {
              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              em->setRobustKernel(rk);
              rk->setDelta(thHuberMotion);
            }

            Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(3, 3);
            // TODO add the sigmaMotion according to anguler speed or Velocity
            em->setInformation(SigmaMotion * Info);
            optimizer.addEdge(em);
            vpEdgeMotion.push_back(em);
            vpFirstHumanPose.push_back(std::make_pair(*firsthmit, itr));
            vpSecondHumanPose.push_back(std::make_pair(*sechmit, itr));
          }

          firsthmit++;
          sechmit = std::next(firsthmit);
        }

      }

      if (pbStopFlag)
        if (*pbStopFlag)
          return;

      optimizer.initializeOptimization();
      optimizer.optimize(5);

      bool bDebug = false;
      bool bDoMore = true;
      std::cerr << "Do more" << '\n';
      if (bDoMore) {
        // cheeck the outlier of stereo observation
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
          g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
          MapPoint *pMP = vpMapPointEdgeStereo[i];

          if (pMP->isBad())
            continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgeHumanKeys.size(); i < iend; i++) {
          g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgeHumanKeys[i];
          std::pair<MapHumanPose *, int> pairObservation = vpObservationHumanKeys[i];

          // Taking the convention from original local bundle adjustment.
          // Leave the bad human pose estimation into outlier
          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            e->setLevel(1);
          }
          // Disable the robust kernel
          e->setRobustKernel(0);
        }

        // For the rigidity constriant I want the outlier segment to perform without affecting
        // the vertex of segment.
        // TODO, this should be done by a new g2o binary edge, which take key as vertex and segment as measurement
        for (size_t i = 0, iend = vpEdgeRigidity.size(); i < iend; i++) {
          EdgeRigidBodyDouble *e = vpEdgeRigidity[i];

          if (e->chi2() > thRanSacRigidity) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
          LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
          if (e->chi2() > thRanSacMotion) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        if (bDebug) {
          ofstream Motion;
          ofstream Segment;
          char frame_index_c[256];
          sprintf(frame_index_c, "%d", pKF->mnId);

          std::string segmentfile = std::string(frame_index_c) + "Segment.txt";
          std::string motionfile = std::string(frame_index_c) + "RanMotion.txt";

          Motion.open(motionfile);
          Segment.open(segmentfile);

          // Segmentt
          for (size_t i = 0, iend = vpEdgeRigidity.size(); i < iend; i++) {
            EdgeRigidBodyDouble *e = vpEdgeRigidity[i];
            std::pair<MapHumanPose *, int> pairsegment = vpSegmentRigidity[i];
            Segment << pairsegment.first->mnId << " " << pairsegment.second << " " << e->chi2() << "\n";
          }

          for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
            LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
            // std::cerr << "the chi2 o f "<<e->chi2() << '\n';
            std::pair<MapHumanPose *, int> pairfirst = vpFirstHumanPose[i];
            std::pair<MapHumanPose *, int> pairsecond = vpSecondHumanPose[i];

            Motion << pairfirst.first->mnTrackId << " " << pairfirst.first->mnId << " " << pairsecond.first->mnId << " "
                   << e->chi2() << "\n";
          }
        }

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

        if (bDebug) {
          ofstream Motion;
          char frame_index_c[256];
          sprintf(frame_index_c, "%d", pKF->mnId);

          std::string motionfile = std::string(frame_index_c) + "SacMotion.txt";

          Motion.open(motionfile);

          for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
            LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
            // std::cerr << "the chi2 o f "<<e->chi2() << '\n';
            std::pair<MapHumanPose *, int> pairfirst = vpFirstHumanPose[i];
            std::pair<MapHumanPose *, int> pairsecond = vpSecondHumanPose[i];

            Motion << pairfirst.first->mnTrackId << " " << pairfirst.first->mnId << " " << pairsecond.first->mnId << " "
                   << e->chi2() << "\n";
          }
        }
      }
      std::cerr << "done" << std::endl;

      // Get Map Mutex
      unique_lock<mutex> lock(pMap->mMutexMapUpdate);

      // rule out the outlier for motion constrint
      for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
        LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
        std::pair<MapHumanPose *, int> pairfirst = vpFirstHumanPose[i];
        // TODO set the lost tracking to l
        if (e->chi2() > thRanSacMotion) {
          int keypairid = pairfirst.second;
          MapHumanKey *pMapHumanKey = pairfirst.first->mvHumanKeyPos[keypairid];
          pMapHumanKey->bIsLost = true;
          pairfirst.first->mpRefHMT->mnBadTrack++;
        }
      }

      // Rule out the outlier for segment
      // TODO The relationship between human pose and segment isbad or optimized
      // TODO add the fixed edge for the
      for (size_t i = 0, iend = vpEdgeRigidity.size(); i < iend; i++) {
        EdgeRigidBodyDouble *e = vpEdgeRigidity[i];
        std::pair<MapHumanPose *, int> pairsegment = vpSegmentRigidity[i];
        MapHumanPose *mMapHumanPose = pairsegment.first;
        int idHumanKeyPair = pairsegment.second;

        if (e->chi2() > thRanSacRigidity) {
          mMapHumanPose->mvHumanKeyPair[idHumanKeyPair].bIsBad = true;
          int firstid = pMap->body1[idHumanKeyPair];
          int secondid = pMap->body2[idHumanKeyPair];

          mMapHumanPose->mvHumanKeyPos[firstid]->bIsFirstBad = true;
          mMapHumanPose->mvHumanKeyPos[secondid]->bIsSecondBad = true;
        } else
          mMapHumanPose->mvHumanKeyPair[idHumanKeyPair].bOptimized = true;
      }

      // Rule out the outlier for human keys
      // Save the result back to the Map Human Keys
      int count = 0;
      int count2 = 0;
      for (size_t i = 0, iend = vpEdgeHumanKeys.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgeHumanKeys[i];
        std::pair<MapHumanPose *, int> pairObservation = vpObservationHumanKeys[i];
        int observationid = pairObservation.second;
        MapHumanPose *pMapHumanPose = pairObservation.first;

        // TODO when the MapHumanPose will be earsed?
        if (pMapHumanPose->isEarsed)
          continue;

        MapHumanKey *pHumanKey = pMapHumanPose->mvHumanKeyPos[observationid];

        if ((pHumanKey->bIsFirstBad) && (pHumanKey->bIsSecondBad)) {
          count++;
          pHumanKey->bIsBad = true;
        }

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          count++;
          pHumanKey->bIsBad = true;
        } else
          count2++;

        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pHumanKey->mnId + maxMTid + 1));
        // In case for error
        if (vPoint != nullptr) {
          pMapHumanPose->SetHumanKeyPos(observationid, Converter::toCvMat(vPoint->estimate()), true);
        }
      }
      std::cerr << "bad human key observation " << count << ", good human key observation " << count2 << '\n';

      vector<pair<KeyFrame *, MapPoint *> > vToErase;
      // vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

      // Check inlier observations
      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
          continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          KeyFrame *pKFi = vpEdgeKFStereo[i];
          vToErase.push_back(make_pair(pKFi, pMP));
        }
      }
      std::cerr << "the length of the map point to earse " << vToErase.size() << '\n';

      if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
          KeyFrame *pKFi = vToErase[i].first;
          MapPoint *pMPi = vToErase[i].second;
          pKFi->EraseMapPointMatch(pMPi);
          pMPi->EraseObservation(pKFi);
        }
      }

      //Keyframes
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
      }

      //Points
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
      }

      // MatMotion
      // Set the flag to isoptimized if it is not a outlier
      // TODO check if the trajectory is outlier
      // TODO check the ratios of the outlier position
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        int id = (*htit)->mnId + maxRGid + 1;
        VertexSE3 *vMotion = static_cast<VertexSE3 *>(optimizer.vertex(id));
        Isometry3 eigenMotion = vMotion->estimate();
        cv::Mat matMotion = Converter::toCvMat(eigenMotion.matrix());
        (*htit)->isOptimized = true;
        (*htit)->mTMotion = matMotion;
        pMap->msetOptimizedTrackID.insert((*htit)->mnTrackID);
      }

      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        vector<MapHumanPose *> vHumanPoses = (*htit)->GetMapHumanTrajectory();
        for (vector<MapHumanPose *>::iterator hmit = vHumanPoses.begin(); hmit != vHumanPoses.end(); hmit++) {
          MapHumanPose *pMapHumanPose = *hmit;
          for (int itr = 0; itr < pMap->mnbodyparts; itr++) {
            // dealling with the humanpose id
            MapHumanKey *pMapKey = pMapHumanPose->mvHumanKeyPos[itr];
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMapKey->mnId + maxMTid + 1));
            if (vPoint != nullptr) {
              pMapHumanPose->SetHumanKeyPos(itr, Converter::toCvMat(vPoint->estimate()), true);
            }
          }
        }
      }


    }

// Thus is the Fast method!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111
    void Optimizer::LocalBundleAdjustmentHumanTrajactory(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                                         float SigmaStatic, float SigmaHuman, float SigmaRigidity,
                                                         float SigmaMotion, float thRanSacMotion,
                                                         float thRanSacRigidity) {


      int nLocalBAFirstIteration = 5;
      int nLocalBASecondIteration = 10;
      //Here the local bundle adjustment taking from human trajectory
      const float thHuberStereo = sqrt(7.815);
      const float thHuberMotion = sqrt(thRanSacMotion);
      const float thHuberRigidity = sqrt(thRanSacRigidity);
      std::cerr << "thRanSacMotion: " << thRanSacMotion << "SigmaMotion: " << SigmaMotion << '\n';

      list<KeyFrame *> lLocalKeyFrames;
      lLocalKeyFrames.push_back(pKF);
      pKF->mnBALocalForHM = pKF->mnId;

      // Enqueue Local KF based on Covisibility graph
      const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
      for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForHM = pKF->mnId;
        if (!pKFi->isBad())
          lLocalKeyFrames.push_back(pKFi);
      }

      vector<MapHumanPose *> vpMapHumanPoses = pKF->mvpMapHumanPoses;
      list<MapHumanTrajectory *> lLocalMapHumanTrajactories;

      // Now input every huamn trajactory regardless the possitbility of a bad traj
      for (vector<MapHumanPose *>::iterator hmit = vpMapHumanPoses.begin(); hmit != vpMapHumanPoses.end(); hmit++) {
        if (*hmit) {
          int nTrackID = (*hmit)->mnTrackId;
          // find the trajactory through id from mmaplocal
          MapHumanTrajectory *pMapHumanTrajectory = pMap->GetMapHumanTrajectory(nTrackID);
          if (!pMapHumanTrajectory) continue;
          // Set the local BA id for the later fixed camera
          pMapHumanTrajectory->mnBALocalForHM = pKF->mnId;
          // Check the length of the MapHumanTrajectory, if it is long enough, use it!
          if (pMapHumanTrajectory->mnHumanPoses > pMap->thLongTrajectory)
            // if (pMapHumanTrajectory->mnBadTrack/(pMapHumanTrajectory->mnHumanPoses*5) < 0.2)
            lLocalMapHumanTrajactories.push_back(pMapHumanTrajectory);
        }
      }

      // Local MapPoints seen in Local KeyFrames
      // Set the variables of mnBALocalForHM for the local map points
      // Later enque the lFixedCamera based on observations
      list<MapPoint *> lLocalMapPoints;
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
          MapPoint *pMP = *vit;
          if (pMP)
            if (!pMP->isBad())
              // The flag to check if the point has enqueued
              if (pMP->mnBALocalForHM != pKF->mnId) {
                lLocalMapPoints.push_back(pMP);
                pMP->mnBALocalForHM = pKF->mnId;
              }
        }
      }

      list<KeyFrame *> lFixedCameras;
      // Fixed camera from local map points
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
          KeyFrame *pKFi = mit->first;

          if (pKFi->mnBALocalForHM != pKF->mnId && pKFi->mnBAFixedForHM != pKF->mnId) {
            pKFi->mnBAFixedForHM = pKF->mnId;

            if (!pKFi->isBad())
              lFixedCameras.push_back(pKFi);
          }
        }
      }

      list<MapHumanPose *> lLocalMapHumanPose;
      // Set the vertex Human Pose
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        vector<MapHumanPose *> vHumanPoses = (*htit)->GetMapHumanTrajectory();
        for (vector<MapHumanPose *>::iterator hmit = vHumanPoses.begin(); hmit != vHumanPoses.end(); hmit++) {
          MapHumanPose *pMapHumanPose = (*hmit);
          if (pMapHumanPose->mpRefKF->mnBALocalForHM == pKF->mnId ||
              pMapHumanPose->mpRefKF->mnBAFixedForHM == pKF->mnId)
            lLocalMapHumanPose.push_back(*hmit);
        }
      }


      // The total number of stereo observation, not include the HumanTrajactory, not include the lFixedCameras that comming from lFixedLocalMapHumanTrajactories
      const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();
      // This is the group of stereo observation
      vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
      vpEdgesStereo.reserve(nExpectedSize);

      vector<KeyFrame *> vpEdgeKFStereo;
      vpEdgeKFStereo.reserve(nExpectedSize);

      vector<MapPoint *> vpMapPointEdgeStereo;
      vpMapPointEdgeStereo.reserve(nExpectedSize);

      // The ope group of Human Key points observation
      // Save the pointer of HumanKeys seems easier, but I might need moreeration.
      vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgeHumanKeys;
      vector<std::pair<MapHumanPose *, int>> vpObservationHumanKeys;

      // Here to save the vertex and edge for rigidity constraint; one corresponding to one
      // A simple solution will be saving the keypair and the MapHumanPose
      vector<EdgeRigidBodyDouble *> vpEdgeRigidity;
      vector<std::pair<MapHumanPose *, int>> vpSegmentRigidity;

      // The edge for motion constraint and the
      vector<LandmarkMotionTernaryEdge *> vpEdgeMotion;
      vector<std::pair<MapHumanPose *, int>> vpFirstHumanPose;
      vector<std::pair<MapHumanPose *, int>> vpSecondHumanPose;

      /*DEBUG
      std::cerr << "the length of lLocalKeyFrames" << lLocalKeyFrames.size() << '\n';
      std::cerr << "the length of lFixedCameras" << lFixedCameras.size() << '\n';
      std::cerr << "the length of lLocalMapPoints" << lLocalMapPoints.size() << '\n';
      std::cerr << "the length of lLocalMapHumanTrajactories " << lLocalMapHumanTrajactories.size() << '\n';
      std::cerr << "the expected size " << nExpectedSize << '\n';
      */

      //set up optimizer
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolverX::LinearSolverType *linearSolver; // BlockSolverX instead of BlockSolver63
      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
      g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      long unsigned maxKFid = 0;
      long unsigned maxMPid = 0;
      long unsigned maxRGid = 0;
      long unsigned maxHPid = 0;
      long unsigned maxMTid = 0;


      // Set the KeyFrame in the lLocalKeyFrames
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);  // TODO Why the KF Node is fixed in BA if the mnID is 0?
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
          maxKFid = pKFi->mnId;
      }

      // Set Fixed KeyFrame Verteices
      for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
          maxKFid = pKFi->mnId;
      }

      std::cerr << "the maxKFid " << maxKFid << '\n';

      // Enqueue the observation of the local map points.
      // Set the edges for observation, This should be the same as the previous setup
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        if (id > maxMPid)
          maxMPid = id;

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();
        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
          KeyFrame *pKFi = mit->first;

          if (!pKFi->isBad()) {
            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKFi->mvuRight[mit->second];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            // DEBUG to see if any null ptr in KFMP
            g2o::VertexSBAPointXYZ *vPoint1 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id));
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
            if ((vPoint1 == nullptr) || (vSE3 == nullptr)) {
              std::cerr << "null ptr KFMP" << id << " " << pKFi->mnId << std::endl;
              continue;
            }

            g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);

            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;
            e->bf = pKFi->mbf;

            // Save the vertex of KF, the map points and edges.
            // This is the same as previous KeyFrame
            optimizer.addEdge(e);
            vpEdgesStereo.push_back(e);
            vpEdgeKFStereo.push_back(pKFi);
            vpMapPointEdgeStereo.push_back(pMP);
          }
        }
      }
      std::cerr << "max MP " << maxMPid << '\n';

      // set vertex rigidty in the local map human trajactories,
      // set up based on the mvRigidBodys
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        MapHumanTrajectory *pHMT = (*htit);
        std::vector<Rigidbody> vRigidity = (*htit)->mvRigidBodys;
        for (std::vector<Rigidbody>::const_iterator itRigid = vRigidity.begin();
             itRigid != vRigidity.end(); itRigid++) {
          VertexDistanceDouble *vRigid = new VertexDistanceDouble();
          int id = maxMPid + itRigid->mnId + 1;
          vRigid->setId(id);

          // if the segemnt is optimized, I would like to take it for initialization
          if (itRigid->isBad && !itRigid->isOptimized) {
            vRigid->setEstimate(0);
          } else {
            vRigid->setEstimate(itRigid->mnDistance);
          }
          optimizer.addVertex(vRigid);

          if (id > maxRGid)
            maxRGid = id;
        }
      }
      std::cerr << "maxRGid " << maxRGid << '\n';

      // Set the vertex of the motion
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        int id = (*htit)->mnId + maxRGid + 1;
        if (id > maxMTid)
          maxMTid = id;
        VertexSE3 *vSE3 = new VertexSE3();
        Isometry3 estimation = Isometry3::Identity();
        if ((*htit)->isOptimized)
          Isometry3 estimation(Converter::toEigenMatrix((*htit)->mTMotion));
        vSE3->setEstimate(estimation);
        vSE3->setId(id);
        optimizer.addVertex(vSE3);
      }
      std::cerr << "maxMTid " << maxMTid << '\n';

      // Set the vertex Human Pose and the edges KeyFrame
      // Adding vertex for HumanPose, edges for human key point observation and edges for rigidity
      for (list<MapHumanPose *>::iterator hmit = lLocalMapHumanPose.begin(); hmit != lLocalMapHumanPose.end(); hmit++) {
        MapHumanPose *pMapHumanpose = *hmit;
        if (pMapHumanpose->isEarsed)
          continue;

        // HumanKey will not be excluded if key isBad in the initializaion. Because I want them to be fixed
        // during the fixed inlier optimization
        // The mnbodyparts is 14, iterate all the possible key points on human
        for (int itr = 0; itr < pMap->mnbodyparts; itr++) {
          // dealing with the human pose id
          MapHumanKey *pMapKey = pMapHumanpose->mvHumanKeyPos[itr];
          int id = pMapKey->mnId + maxMTid + 1;
          if (id > maxHPid) maxHPid = id;

          // setting up the vertex for human key
          // If not a mapping mod, let's skip the bad key frame
          if ((pMapKey->bIsBad) && (pMap->mbVOOnlyFlag)) continue;

          g2o::VertexSBAPointXYZ *vHumanKey = new g2o::VertexSBAPointXYZ();
          vHumanKey->setEstimate(Converter::toVector3d(pMapKey->WorldPos));
          if (pMapKey->WorldPos.empty()) std::cerr << "empty id" << id << '\n';
          vHumanKey->setId(id);
          optimizer.addVertex(vHumanKey);

          // if the key is bad, I will not take this observation into edges,
          // Leave it for the rigidity and the motion constraint
          if (pKF->isBad()) continue;

          // if the human pose is observed by the keyframe, we can constrain the obs
          if (pMapHumanpose->mbIsInKeyFrame) {
            KeyFrame *pKFHmp = (pMapHumanpose->mObservations).first;
            int obsid = pMapHumanpose->mObservations.second;//the mapping for corresponding position

            Eigen::Matrix<double, 3, 1> obs;
            human_pose mHuman = pKFHmp->mvHumanPoses[obsid];
            const cv::KeyPoint kpUn = mHuman.vHumanKeyPoints[itr];
            const float kp_ur = mHuman.vHumanKeyPointsRight[itr].pt.x;
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::OptimizableGraph::Vertex *vPoint = optimizer.vertex(id);              // g2o::VertexSBAPointXYZ*
            g2o::OptimizableGraph::Vertex *vSE3 = optimizer.vertex(pKFHmp->mnId);  // g2o::VertexSE3Expmap*
            if ((vPoint == nullptr) || (vSE3 == nullptr)) continue;

            // Add project constraint to hmp
            g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();
            e->setVertex(0, optimizer.vertex(id));
            e->setVertex(1, optimizer.vertex(pKFHmp->mnId));
            e->setMeasurement(obs);

            // TODO the information matrix, can be initiate with the probability from human poses
            const float &invSigma2 = SigmaHuman;
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            e->fx = pKFHmp->fx;
            e->fy = pKFHmp->fy;
            e->cx = pKFHmp->cx;
            e->cy = pKFHmp->cy;
            e->bf = pKFHmp->mbf;

            optimizer.addEdge(e);

            // Record the edges, prepare for the ransacs
            vpEdgeHumanKeys.push_back(e);
            vpObservationHumanKeys.push_back(std::make_pair(pMapHumanpose, obsid));
          }
        }

        // Adding rigidity constraint
        // Doing this with idx since we need to save the order in
        for (int i = 0; i < pMapHumanpose->mvHumanKeyPair.size(); i++) {

          HumanKeyPair pairHumanKeys = pMapHumanpose->mvHumanKeyPair[i];

          int id1 = pairHumanKeys.idFirstKey + maxMTid + 1;
          int id2 = pairHumanKeys.idSecondKey + maxMTid + 1;
          int id3 = pairHumanKeys.idDistance + maxMPid + 1; // May have problem

          g2o::VertexSBAPointXYZ *vHumanKey1 = dynamic_cast<g2o::VertexSBAPointXYZ *>( optimizer.vertex(id1));
          g2o::VertexSBAPointXYZ *vHumanKey2 = dynamic_cast<g2o::VertexSBAPointXYZ *>( optimizer.vertex(id2));

          if ((vHumanKey1 == nullptr) || (vHumanKey2 == nullptr)) {
            continue;
          }

          // if we only consider the VO, many part can be eliminated
          if (pMap->mbVOOnlyFlag) {
            if (pairHumanKeys.bIsBad)
              continue;
          }

          EdgeRigidBodyDouble *edgeRigid = new EdgeRigidBodyDouble();
          edgeRigid->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id1)));
          edgeRigid->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id2)));
          edgeRigid->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id3)));

          Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(1, 1);
          Info(0, 0) = SigmaRigidity;
          edgeRigid->setInformation(Info);

          // TODO find the appropriate huber
          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          edgeRigid->setRobustKernel(rk);
          rk->setDelta(thRanSacRigidity);

          optimizer.addEdge(edgeRigid);
          vpEdgeRigidity.push_back(edgeRigid);
          // std::pair<*MapHumanPose,int> pairRigidSegment (pMapHumanpose, i);
          vpSegmentRigidity.push_back(std::make_pair(pMapHumanpose, i));
        }
      }


      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        vector<MapHumanPose *> vHumanPoses = (*htit)->GetMapHumanTrajectory();
        // Motion constraint
        int motionid = (*htit)->mnId + maxRGid + 1;
        vector<MapHumanPose *>::iterator firsthmit = vHumanPoses.begin();
        vector<MapHumanPose *>::iterator sechmit = std::next(firsthmit);
        // (*firsthmit)->isLost=true;
        while (sechmit != vHumanPoses.end()) {
          // If the human pose if bad, we add this value, if greater than a thresh, we skip it
          int bad_tracking_count = 0;
          if ((*firsthmit)->mpRefKF->mnBALocalForHM == pKF->mnId ||
              (*firsthmit)->mpRefKF->mnBAFixedForHM == pKF->mnId) {
            double time_range = (*sechmit)->mTimeStamp - (*firsthmit)->mTimeStamp;

            for (int i = 0; i < pMap->mimainskleton; i++) {
              int itr = pMap->mainskleton[i];

              MapHumanKey *pfisrtMapKey = (*firsthmit)->mvHumanKeyPos[itr];
              int id1 = pfisrtMapKey->mnId + maxMTid + 1;
              MapHumanKey *pMapKey = (*sechmit)->mvHumanKeyPos[itr];
              int id2 = pMapKey->mnId + maxMTid + 1;

              // bool isLost = pfisrtMapKey->bIsLost;
              bool isLost = false;

              LandmarkMotionTernaryEdge *em = new LandmarkMotionTernaryEdge();

              g2o::VertexSBAPointXYZ *vPoint1 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id1));
              g2o::VertexSBAPointXYZ *vPoint2 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id2));
              VertexSE3 *vSE3 = static_cast<VertexSE3 *>(optimizer.vertex(motionid));

              // If the points is bad here it will be a null ptr
              if ((vSE3 == nullptr) || (vPoint1 == nullptr) || (vPoint2 == nullptr)) {
                // else enqueue everyone
                bad_tracking_count++;
                continue;
              }

              em->setVertex(0, optimizer.vertex(id1));
              em->setVertex(1, optimizer.vertex(id2));
              em->setVertex(2, optimizer.vertex(motionid));
              em->delta_t = time_range;
              em->setMeasurement(Eigen::Vector3d(0, 0, 0));

              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              em->setRobustKernel(rk);
              rk->setDelta(thHuberMotion);

              Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(3, 3);
              // TODO add the sigmaMotion according to anguler speed or Velocity
              em->setInformation(SigmaMotion * Info);
              optimizer.addEdge(em);
              vpEdgeMotion.push_back(em);
              vpFirstHumanPose.push_back(std::make_pair(*firsthmit, itr));
              vpSecondHumanPose.push_back(std::make_pair(*sechmit, itr));
            }
          }
          // If all key points is regarded as bad, skip the motion constraint connection

          if (bad_tracking_count < pMap->mimainskleton)
            firsthmit++;
          sechmit++;
        }

      }

      if (pbStopFlag && *pbStopFlag) return;

      optimizer.initializeOptimization();
      optimizer.optimize(nLocalBAFirstIteration);

      bool bDebug = false;
      bool bDoMore = true;
      std::cerr << "Do more" << '\n';
      if (bDoMore) {
        // cheeck the outlier of stereo observation
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
          g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
          MapPoint *pMP = vpMapPointEdgeStereo[i];

          if (pMP->isBad())
            continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgeHumanKeys.size(); i < iend; i++) {
          g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgeHumanKeys[i];
          std::pair<MapHumanPose *, int> pairObservation = vpObservationHumanKeys[i];

          // Taking the convention from original local bundle adjustment.
          // Leave the bad human pose estimation into outlier
          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            e->setLevel(1);
          }
          // Disable the robust kernel
          e->setRobustKernel(0);
        }

        // For the rigidity constriant I want the outlier segment to perform without affecting
        // the vertex of segment.
        // TODO, this should be done by a new g2o binary edge, which take key as vertex and segment as measurement
        for (size_t i = 0, iend = vpEdgeRigidity.size(); i < iend; i++) {
          EdgeRigidBodyDouble *e = vpEdgeRigidity[i];

          if (e->chi2() > thRanSacRigidity) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
          LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
          if (e->chi2() > thRanSacMotion) {
            e->setLevel(1);
          }

          e->setRobustKernel(0);
        }

        if (bDebug) {
          ofstream Motion;
          ofstream Segment;
          char frame_index_c[256];
          sprintf(frame_index_c, "%d", pKF->mnId);

          std::string segmentfile = std::string(frame_index_c) + "Segment.txt";
          std::string motionfile = std::string(frame_index_c) + "RanMotion.txt";

          Motion.open(motionfile);
          Segment.open(segmentfile);

          // Segment
          for (size_t i = 0, iend = vpEdgeRigidity.size(); i < iend; i++) {
            EdgeRigidBodyDouble *e = vpEdgeRigidity[i];
            std::pair<MapHumanPose *, int> pairsegment = vpSegmentRigidity[i];
            Segment << pairsegment.first->mnId << " " << pairsegment.second << " " << e->chi2() << "\n";
          }

          for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
            LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
            std::pair<MapHumanPose *, int> pairfirst = vpFirstHumanPose[i];
            std::pair<MapHumanPose *, int> pairsecond = vpSecondHumanPose[i];

            Motion << pairfirst.first->mnTrackId << " " << pairfirst.first->mnId << " " << pairsecond.first->mnId << " "
                   << e->chi2() << "\n";
          }
        }

        optimizer.initializeOptimization(0);
        optimizer.optimize(nLocalBASecondIteration);

        if (bDebug) {
          ofstream Motion;
          char frame_index_c[256];
          sprintf(frame_index_c, "%d", pKF->mnId);

          std::string motionfile = std::string(frame_index_c) + "SacMotion.txt";

          Motion.open(motionfile);

          for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
            LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
            // std::cerr << "the chi2 o f "<<e->chi2() << '\n';
            std::pair<MapHumanPose *, int> pairfirst = vpFirstHumanPose[i];
            std::pair<MapHumanPose *, int> pairsecond = vpSecondHumanPose[i];

            Motion << pairfirst.first->mnTrackId << " " << pairfirst.first->mnId << " " << pairsecond.first->mnId << " "
                   << e->chi2() << "\n";
          }
        }
      }

      // Get Map Mutex
      unique_lock<mutex> lock(pMap->mMutexMapUpdate);

      // rule out the outlier for motion constrint
      for (size_t i = 0, iend = vpEdgeMotion.size(); i < iend; i++) {
        LandmarkMotionTernaryEdge *e = vpEdgeMotion[i];
        std::pair<MapHumanPose *, int> pairfirst = vpFirstHumanPose[i];
        // TODO set the lost tracking to l
        if (e->chi2() > thRanSacMotion) {
          int keypairid = pairfirst.second;
          MapHumanKey *pMapHumanKey = pairfirst.first->mvHumanKeyPos[keypairid];
          pMapHumanKey->bIsLost = true;
          pairfirst.first->mpRefHMT->mnBadTrack++;
        }
      }

      // Rule out the outlier for segment
      // TODO The relationship between human pose and segment isbad or optimized
      // TODO add the fixed edge for the
      for (size_t i = 0, iend = vpEdgeRigidity.size(); i < iend; i++) {
        EdgeRigidBodyDouble *e = vpEdgeRigidity[i];
        std::pair<MapHumanPose *, int> pairsegment = vpSegmentRigidity[i];
        MapHumanPose *mMapHumanPose = pairsegment.first;
        int idHumanKeyPair = pairsegment.second;

        if (e->chi2() > thRanSacRigidity) {
          mMapHumanPose->mvHumanKeyPair[idHumanKeyPair].bIsBad = true;
          int firstid = pMap->body1[idHumanKeyPair];
          int secondid = pMap->body2[idHumanKeyPair];

          mMapHumanPose->mvHumanKeyPos[firstid]->bIsFirstBad = true;
          mMapHumanPose->mvHumanKeyPos[secondid]->bIsSecondBad = true;
        } else
          mMapHumanPose->mvHumanKeyPair[idHumanKeyPair].bOptimized = true;
      }

      // Rule out the outlier for human keys
      // Save the result back to the Map Human Keys
      int count = 0;
      int count2 = 0;
      for (size_t i = 0, iend = vpEdgeHumanKeys.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgeHumanKeys[i];
        std::pair<MapHumanPose *, int> pairObservation = vpObservationHumanKeys[i];
        int observationid = pairObservation.second;
        MapHumanPose *pMapHumanPose = pairObservation.first;

        // TODO when the MapHumanPose will be earsed?
        if (pMapHumanPose->isEarsed)
          continue;

        MapHumanKey *pHumanKey = pMapHumanPose->mvHumanKeyPos[observationid];

        if ((pHumanKey->bIsFirstBad) && (pHumanKey->bIsSecondBad)) {
          count++;
          pHumanKey->bIsBad = true;
        }

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          count++;
          pHumanKey->bIsBad = true;
        } else
          count2++;

        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pHumanKey->mnId + maxMTid + 1));
        // In case for error
        if (vPoint != nullptr) {
          pMapHumanPose->SetHumanKeyPos(observationid, Converter::toCvMat(vPoint->estimate()), true);
          // pHumanKey->WorldPos = Converter::toCvMat(vPoint->estimate());
        }
        // TODO shall we  consider the outlier?
      }
      std::cerr << "bad human key observation " << count << "good human key observation" << count2 << '\n';

      vector<pair<KeyFrame *, MapPoint *> > vToErase;
      // vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

      // Check inlier observations
      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
          continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          KeyFrame *pKFi = vpEdgeKFStereo[i];
          vToErase.push_back(make_pair(pKFi, pMP));
        }
      }
      std::cerr << "the length of the map point to earse" << vToErase.size() << '\n';

      if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
          KeyFrame *pKFi = vToErase[i].first;
          MapPoint *pMPi = vToErase[i].second;
          pKFi->EraseMapPointMatch(pMPi);
          pMPi->EraseObservation(pKFi);
        }
      }

      //Keyframes
      for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
      }

      //Points
      for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
      }

      // MatMotion
      // Set the flag to isoptimized if it is not a outlier
      // TODO check if the trajectory is outlier
      // TODO check the ratios of the outlier position
      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        int id = (*htit)->mnId + maxRGid + 1;
        VertexSE3 *vMotion = static_cast<VertexSE3 *>(optimizer.vertex(id));
        Isometry3 eigenMotion = vMotion->estimate();
        cv::Mat matMotion = Converter::toCvMat(eigenMotion.matrix());
        (*htit)->isOptimized = true;
        (*htit)->mTMotion = matMotion;
        pMap->msetOptimizedTrackID.insert((*htit)->mnTrackID);
      }

      for (list<MapHumanTrajectory *>::iterator htit = lLocalMapHumanTrajactories.begin();
           htit != lLocalMapHumanTrajactories.end(); htit++) {
        vector<MapHumanPose *> vHumanPoses = (*htit)->GetMapHumanTrajectory();
        for (vector<MapHumanPose *>::iterator hmit = vHumanPoses.begin(); hmit != vHumanPoses.end(); hmit++) {
          MapHumanPose *pMapHumanPose = *hmit;
          for (int itr = 0; itr < pMap->mnbodyparts; itr++) {
            // dealling with the humanpose id
            MapHumanKey *pMapKey = pMapHumanPose->mvHumanKeyPos[itr];
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMapKey->mnId + maxMTid + 1));
            if (vPoint != nullptr) {
              pMapHumanPose->SetHumanKeyPos(itr, Converter::toCvMat(vPoint->estimate()), true);
            }
          }
        }
      }


    }


    void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                           const bool &bFixScale) {
      // Setup optimizer
      g2o::SparseOptimizer optimizer;
      optimizer.setVerbose(false);
      g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
          new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
      g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

      solver->setUserLambdaInit(1e-16);
      optimizer.setAlgorithm(solver);

      const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
      const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

      const unsigned int nMaxKFid = pMap->GetMaxKFid();

      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid + 1);
      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);
      vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

      const int minFeat = 100;

      // Set KeyFrame vertices
      for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
          continue;
        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if (it != CorrectedSim3.end()) {
          vScw[nIDi] = it->second;
          VSim3->setEstimate(it->second);
        } else {
          Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
          Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
          g2o::Sim3 Siw(Rcw, tcw, 1.0);
          vScw[nIDi] = Siw;
          VSim3->setEstimate(Siw);
        }

        if (pKF == pLoopKF)
          VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;
      }


      set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

      const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

      // Set Loop edges
      for (map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end();
           mit != mend; mit++) {
        KeyFrame *pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame *> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end();
             sit != send; sit++) {
          const long unsigned int nIDj = (*sit)->mnId;
          if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
            continue;

          const g2o::Sim3 Sjw = vScw[nIDj];
          const g2o::Sim3 Sji = Sjw * Swi;

          g2o::EdgeSim3 *e = new g2o::EdgeSim3();
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
          e->setMeasurement(Sji);

          e->information() = matLambda;

          optimizer.addEdge(e);

          sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
        }
      }

      // Set normal edges
      for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        KeyFrame *pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if (iti != NonCorrectedSim3.end())
          Swi = (iti->second).inverse();
        else
          Swi = vScw[nIDi].inverse();

        KeyFrame *pParentKF = pKF->GetParent();

        // Spanning tree edge
        if (pParentKF) {
          int nIDj = pParentKF->mnId;

          g2o::Sim3 Sjw;

          LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

          if (itj != NonCorrectedSim3.end())
            Sjw = itj->second;
          else
            Sjw = vScw[nIDj];

          g2o::Sim3 Sji = Sjw * Swi;

          g2o::EdgeSim3 *e = new g2o::EdgeSim3();
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
          e->setMeasurement(Sji);

          e->information() = matLambda;
          optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
        for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
          KeyFrame *pLKF = *sit;
          if (pLKF->mnId < pKF->mnId) {
            g2o::Sim3 Slw;

            LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

            if (itl != NonCorrectedSim3.end())
              Slw = itl->second;
            else
              Slw = vScw[pLKF->mnId];

            g2o::Sim3 Sli = Slw * Swi;
            g2o::EdgeSim3 *el = new g2o::EdgeSim3();
            el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
            el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
            el->setMeasurement(Sli);
            el->information() = matLambda;
            optimizer.addEdge(el);
          }
        }

        // Covisibility graph edges
        const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
          KeyFrame *pKFn = *vit;
          if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
            if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
              if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
                continue;

              g2o::Sim3 Snw;

              LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

              if (itn != NonCorrectedSim3.end())
                Snw = itn->second;
              else
                Snw = vScw[pKFn->mnId];

              g2o::Sim3 Sni = Snw * Swi;

              g2o::EdgeSim3 *en = new g2o::EdgeSim3();
              en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
              en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
              en->setMeasurement(Sni);
              en->information() = matLambda;
              optimizer.addEdge(en);
            }
          }
        }
      }

      // Optimize!
      optimizer.initializeOptimization();
      optimizer.optimize(20);

      unique_lock<mutex> lock(pMap->mMutexMapUpdate);

      // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
      for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *= (1. / s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

        pKFi->SetPose(Tiw);
      }

      // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
      for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        MapPoint *pMP = vpMPs[i];

        if (pMP->isBad())
          continue;

        int nIDr;
        if (pMP->mnCorrectedByKF == pCurKF->mnId) {
          nIDr = pMP->mnCorrectedReference;
        } else {
          KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
          nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
      }
    }

    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12,
                                const float th2, const bool bFixScale) {
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolverX::LinearSolverType *linearSolver;

      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

      g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      // Calibration
      const cv::Mat &K1 = pKF1->mK;
      const cv::Mat &K2 = pKF2->mK;

      // Camera poses
      const cv::Mat R1w = pKF1->GetRotation();
      const cv::Mat t1w = pKF1->GetTranslation();
      const cv::Mat R2w = pKF2->GetRotation();
      const cv::Mat t2w = pKF2->GetTranslation();

      // Set Sim3 vertex
      g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
      vSim3->_fix_scale = bFixScale;
      vSim3->setEstimate(g2oS12);
      vSim3->setId(0);
      vSim3->setFixed(false);
      vSim3->_principle_point1[0] = K1.at<float>(0, 2);
      vSim3->_principle_point1[1] = K1.at<float>(1, 2);
      vSim3->_focal_length1[0] = K1.at<float>(0, 0);
      vSim3->_focal_length1[1] = K1.at<float>(1, 1);
      vSim3->_principle_point2[0] = K2.at<float>(0, 2);
      vSim3->_principle_point2[1] = K2.at<float>(1, 2);
      vSim3->_focal_length2[0] = K2.at<float>(0, 0);
      vSim3->_focal_length2[1] = K2.at<float>(1, 1);
      optimizer.addVertex(vSim3);

      // Set MapPoint vertices
      const int N = vpMatches1.size();
      const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
      vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
      vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
      vector<size_t> vnIndexEdge;

      vnIndexEdge.reserve(2 * N);
      vpEdges12.reserve(2 * N);
      vpEdges21.reserve(2 * N);

      const float deltaHuber = sqrt(th2);

      int nCorrespondences = 0;

      for (int i = 0; i < N; i++) {
        if (!vpMatches1[i])
          continue;

        MapPoint *pMP1 = vpMapPoints1[i];
        MapPoint *pMP2 = vpMatches1[i];

        const int id1 = 2 * i + 1;
        const int id2 = 2 * (i + 1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if (pMP1 && pMP2) {
          if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0) {
            g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
            cv::Mat P3D1w = pMP1->GetWorldPos();
            cv::Mat P3D1c = R1w * P3D1w + t1w;
            vPoint1->setEstimate(Converter::toVector3d(P3D1c));
            vPoint1->setId(id1);
            vPoint1->setFixed(true);
            optimizer.addVertex(vPoint1);

            g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
            cv::Mat P3D2w = pMP2->GetWorldPos();
            cv::Mat P3D2c = R2w * P3D2w + t2w;
            vPoint2->setEstimate(Converter::toVector3d(P3D2c));
            vPoint2->setId(id2);
            vPoint2->setFixed(true);
            optimizer.addVertex(vPoint2);
          } else
            continue;
        } else
          continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double, 2, 1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

        g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double, 2, 1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

        g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
      }

      // Optimize!
      optimizer.initializeOptimization();
      optimizer.optimize(5);

      // Check inliers
      int nBad = 0;
      for (size_t i = 0; i < vpEdges12.size(); i++) {
        g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
        if (!e12 || !e21)
          continue;

        if (e12->chi2() > th2 || e21->chi2() > th2) {
          size_t idx = vnIndexEdge[i];
          vpMatches1[idx] = static_cast<MapPoint *>(NULL);
          optimizer.removeEdge(e12);
          optimizer.removeEdge(e21);
          vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
          vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
          nBad++;
        }
      }

      int nMoreIterations;
      if (nBad > 0)
        nMoreIterations = 10;
      else
        nMoreIterations = 5;

      if (nCorrespondences - nBad < 10)
        return 0;

      // Optimize again only with inliers

      optimizer.initializeOptimization();
      optimizer.optimize(nMoreIterations);

      int nIn = 0;
      for (size_t i = 0; i < vpEdges12.size(); i++) {
        g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
        if (!e12 || !e21)
          continue;

        if (e12->chi2() > th2 || e21->chi2() > th2) {
          size_t idx = vnIndexEdge[i];
          vpMatches1[idx] = static_cast<MapPoint *>(NULL);
        } else
          nIn++;
      }

      // Recover optimized Sim3
      g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
      g2oS12 = vSim3_recov->estimate();

      return nIn;
    }

    // one human trajactory one time
    void Optimizer::GlobalBundle(Map *pMap, MapHumanTrajectory *pMHT, float SigmaStatic,
                                 float SigmaHuman, float SigmaRigidity, float SigmaMotion, int nIterations,
                                 float thHuberMotion, bool bRobust) {
      // loading the map point keyframe and huanman pose; Also the idx reference
      vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
      vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
      vector<MapHumanPose *> allHumanPoses = pMHT->GetMapHumanTrajectory();
      std::vector<Rigidbody> RigidBodys = pMHT->mvRigidBodys;
      long unsigned int maxKFid = 0;
      long unsigned int maxMPid = 0;
      long unsigned int maxRGid = 0;
      long unsigned int maxHPid = 0;
      long unsigned int maxMTid = 0;
      long unsigned int nLoopKF = 0;// TODO add on human pose

      vector<bool> vbNotIncludedMP;
      vbNotIncludedMP.resize(vpMP.size());

      const float thHuber2D = sqrt(5.99);
      const float thHuber3D = sqrt(7.815);

      // const bool bRobust = true;

      //set up optimizer
      g2o::SparseOptimizer optimizer;
      std::cerr << "Setting up solver and optimizer" << '\n';
      g2o::BlockSolverX::LinearSolverType *linearSolver; // BlockSolverX instead of BlockSolver63
      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
      g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      // Set KeyFrame vertices
      for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
          continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        // std::cerr << "keyframe vertex: "<<pKF->mnId << '\n';
        vSE3->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKF->mnId > maxKFid)
          maxKFid = pKF->mnId;
      }
      // Set MapPoint vertices
      for (size_t i = 0; i < vpMP.size(); i++) {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
          continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        if (id > maxMPid) maxMPid = id;

        // std::cerr << "mappoint vertex: "<<id << '\n';

        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        // the observation reference to keyframe
        const map<KeyFrame *, size_t> observations = pMP->GetObservations();
        int nEdges = 0;
        //Set Edges // Stereo only
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {
          KeyFrame *pKF = mit->first;
          if (pKF->isBad() || pKF->mnId > maxKFid)
            continue;

          nEdges++;

          const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKF->mvuRight[mit->second];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave] * SigmaStatic;
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          //Always use huber function
          // if(bRobust)
          // {
          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber3D);
          // }

          e->fx = pKF->fx;
          e->fy = pKF->fy;
          e->cx = pKF->cx;
          e->cy = pKF->cy;
          e->bf = pKF->mbf;

          optimizer.addEdge(e);
        }
        // In case a map point have no observation
        if (nEdges == 0) {
          optimizer.removeVertex(vPoint);
          vbNotIncludedMP[i] = true;
        } else {
          vbNotIncludedMP[i] = false;
        }
      }

      // Set rigidity
      // TODO for different human traj

      // Set up the motion edge
      // VertexSE3 which is different to the se3expmap
      int motionEdgeId = ++maxMPid;//TODO
      maxMTid = motionEdgeId;
      VertexSE3 *vSE3 = new VertexSE3();
      Isometry3 estimation(Isometry3::Identity());
      vSE3->setEstimate(estimation);
      vSE3->setId(motionEdgeId);
      optimizer.addVertex(vSE3);

      for (std::vector<Rigidbody>::const_iterator itRigid = RigidBodys.begin();
           itRigid != RigidBodys.end(); itRigid++) {
        VertexDistanceDouble *vRigid = new VertexDistanceDouble();
        int id = maxMPid + itRigid->mnId + 1;
        vRigid->setId(id);
        if (itRigid->isBad) {
          vRigid->setEstimate(0);
        } else {
          vRigid->setEstimate(itRigid->mnDistance);
        }
        optimizer.addVertex(vRigid);
        // std::cerr << "ridig vetex: " <<id<< '\n';
        if (id > maxRGid)
          maxRGid = id;
      }

      //Set Human Pose
      for (size_t MHPid = 0; MHPid < allHumanPoses.size(); MHPid++) {
        MapHumanPose *mMapHumanpose = allHumanPoses[MHPid];
        // Add human pose vertex and KeyFrame observation
        // TODO check the KeyFrame is bad
        if (mMapHumanpose->isEarsed) {//std::cerr << "earsed kf" << '\n';
          continue;
        }
        for (int itr = 0; itr < 14; itr++) {
          //both human key points and human share the same id;
          MapHumanKey *pMapKey = mMapHumanpose->mvHumanKeyPos[itr];
          int id = pMapKey->mnId + maxRGid + 1;
          if (id > maxHPid) maxHPid = id;

          g2o::VertexSBAPointXYZ *vHumanKey = new g2o::VertexSBAPointXYZ();
          bool isBad = pMapKey->bIsBad;
          // std::cerr << "is Bad "<< isBad << '\n';
          if (isBad)
            continue;
            // vHumanKey->setEstimate(Eigen::Vector3d(0,0,0));//TODO a better initial pose
          else
            vHumanKey->setEstimate(Converter::toVector3d(pMapKey->WorldPos));
          vHumanKey->setId(id);

          optimizer.addVertex(vHumanKey);
          // std::cerr << "HumanKeyId: " <<id << '\n';

          // So if there is no a keyframe used in this frame
          // std::cerr << "mMapHumanpose->mbIsInKeyFrame"<<mMapHumanpose->mbIsInKeyFrame << '\n';
          if (mMapHumanpose->mbIsInKeyFrame) {

            int obsid = mMapHumanpose->mObservations.second;//the mapping for corresponding position
            // std::cerr << "obsid"<<obsid << '\n';
            KeyFrame *pKF = (mMapHumanpose->mObservations).first;
            if (pKF->isBad()) continue;

            // Add Observations
            Eigen::Matrix<double, 3, 1> obs;
            // std::cerr << "the length of hmpse"<<pKF->mvHumanPoses.size() << '\n';
            human_pose mHuman = pKF->mvHumanPoses[obsid];

            const cv::KeyPoint kpUn = mHuman.vHumanKeyPoints[itr];
            const float kp_ur = mHuman.vHumanKeyPointsRight[itr].pt.x;
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
            // std::cerr << "obs" <<obs<< '\n';

            // Add project constraint to hmp
            g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
            e->setMeasurement(obs);
            const float &invSigma2 = SigmaHuman;
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            //TODO Add huber function
            e->fx = pKF->fx;
            e->fy = pKF->fy;
            e->cx = pKF->cx;
            e->cy = pKF->cy;
            e->bf = pKF->mbf;

            optimizer.addEdge(e);
          }
        }

        // Add rigidity constraint
        for (std::vector<HumanKeyPair>::iterator itHKP = mMapHumanpose->mvHumanKeyPair.begin();
             itHKP != mMapHumanpose->mvHumanKeyPair.end(); itHKP++) {
          if (itHKP->bIsBad) continue;//TODO add initialization;
          int id1 = itHKP->idFirstKey + maxRGid + 1;
          int id2 = itHKP->idSecondKey + maxRGid + 1;
          int id3 = itHKP->idDistance + maxMPid + 1; // May have problem

          // std::cerr << "id: "<<id1 <<" "<<id2 <<" "<< id3 <<" " << '\n';

          g2o::VertexSBAPointXYZ *vHumanKey1 = dynamic_cast<g2o::VertexSBAPointXYZ *>( optimizer.vertex(id1));
          g2o::VertexSBAPointXYZ *vHumanKey2 = dynamic_cast<g2o::VertexSBAPointXYZ *>( optimizer.vertex(id2));
          EdgeRigidBodyDouble *edgeRigid = new EdgeRigidBodyDouble();
          // std::cerr << "norm: "<< (vHumanKey1->estimate() -vHumanKey2->estimate()).norm() << '\n';

          edgeRigid->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id1)));
          edgeRigid->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id2)));
          edgeRigid->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>( optimizer.vertex(id3)));

          // edgeRigid->computeError_debug();

          Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(1, 1);
          Info(0, 0) = SigmaRigidity;
          edgeRigid->setInformation(Info);
          optimizer.addEdge(edgeRigid);
        }
      }

      // Set Motion Constraint
      // Init the first HP and KF
      if (allHumanPoses.size() > 0) {
        int first_id = 0;
        MapHumanPose *firstMapHumanpose = allHumanPoses[first_id];
        // if (firstMapHumanpose->mbIsInKeyFrame){
        //   KeyFrame* pfirstKF = (firstMapHumanpose->mObservations).first;
        // }
        for (size_t MHPid = first_id + 1; MHPid < allHumanPoses.size(); MHPid++) {
          // not every point //
          MapHumanPose *mMapHumanpose = allHumanPoses[MHPid];
          // Add human pose vertex and KeyFrame observation
          // KeyFrame* pKF = (mMapHumanpose->mObservations).first;
          // double time_range = pKF->mTimeStamp - pfirstKF->mTimeStamp;
          double time_range = mMapHumanpose->mTimeStamp - firstMapHumanpose->mTimeStamp;
          // std::cerr << "time_range: "<<time_range << '\n';

          // it should be 1, 2, 5 11, 8
          int mainskleton[5] = {1, 2, 5, 11, 8};
          for (int i = 0; i < 5; i++) {
            int itr = mainskleton[i];
            LandmarkMotionTernaryEdge *em = new LandmarkMotionTernaryEdge();

            MapHumanKey *pfisrtMapKey = firstMapHumanpose->mvHumanKeyPos[itr];
            int id1 = pfisrtMapKey->mnId + maxRGid + 1;
            MapHumanKey *pMapKey = mMapHumanpose->mvHumanKeyPos[itr];
            int id2 = pMapKey->mnId + maxRGid + 1;
            // std::cerr << "id: "<<id1<<" "<<id2 << '\n';
            // std::cerr << "is bad"<<pMapKey->bIsBad<< " "<< pfisrtMapKey->bIsBad<< '\n';
            if ((pMapKey->bIsBad) || (pfisrtMapKey->bIsBad)) continue;

            g2o::VertexSBAPointXYZ *vPoint1 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id1));
            g2o::VertexSBAPointXYZ *vPoint2 = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id2));
            VertexSE3 *vSE3 = static_cast<VertexSE3 *>(optimizer.vertex(motionEdgeId));
            em->setVertex(0, optimizer.vertex(id1));
            em->setVertex(1, optimizer.vertex(id2));
            em->setVertex(2, optimizer.vertex(motionEdgeId));
            em->delta_t = time_range;
            em->setMeasurement(Eigen::Vector3d(0, 0, 0));
            if (bRobust) {
              g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
              em->setRobustKernel(rk);
              rk->setDelta(thHuberMotion);
            }

            Eigen::MatrixXd Info = Eigen::MatrixXd::Identity(3, 3);
            em->setInformation(SigmaMotion * Info);
            optimizer.addEdge(em);
          }
          // save first HMP
          // pfirstKF = pKF;
          firstMapHumanpose = mMapHumanpose;
        }
      }

      // Optimize!
      optimizer.initializeOptimization();
      optimizer.optimize(nIterations);

      std::cerr << "save back " << '\n';
      //Keyframes
      for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
          continue;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (nLoopKF == 0) {
          pKF->SetPose(Converter::toCvMat(SE3quat));
        } else {
          pKF->mTcwGBA.create(4, 4, CV_32F);
          Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
          pKF->mnBAGlobalForKF = nLoopKF;
        }
      }

      //Points
      for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
          continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
          continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
          pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
          pMP->UpdateNormalAndDepth();
        } else {
          pMP->mPosGBA.create(3, 1, CV_32F);
          Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
          pMP->mnBAGlobalForKF = nLoopKF;
        }
      }

      // Human Pose
      for (size_t MHPid = 0; MHPid < allHumanPoses.size(); MHPid++) {
        MapHumanPose *mMapHumanpose = allHumanPoses[MHPid];
        for (int itr = 0; itr < 14; itr++) {

          //both human key points and human share the same id;
          MapHumanKey *pMapKey = mMapHumanpose->mvHumanKeyPos[itr];
          bool isBad = pMapKey->bIsBad;
          if (isBad)
            continue;
          int id = pMapKey->mnId + maxRGid + 1;
          // std::cerr << "id: "<<id << '\n';
          g2o::VertexSBAPointXYZ *vHMP = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(id));
          cv::Mat optimizedPos = Converter::toCvMat(vHMP->estimate());
          cv::Mat toDebug = allHumanPoses[MHPid]->GetHumanKeyPos(itr);
          // std::cerr << "to - debug " << cv::norm(optimizedPos, toDebug) << '\n';
          mMapHumanpose->SetHumanKeyPos(itr, optimizedPos, true);
        }
      }

      // Set rigidity
      for (std::vector<Rigidbody>::const_iterator itRigid = RigidBodys.begin();
           itRigid != RigidBodys.end(); itRigid++) {
        int id = maxMPid + itRigid->mnId + 1;
        VertexDistanceDouble *vRigid = static_cast<VertexDistanceDouble *>(optimizer.vertex(id));
        // std::cerr << "vRigid.estimate" << vRigid->estimate() << '\n';
        // std::cerr << "before"<< itRigid->mnDistance << '\n';
      }

      VertexSE3 *vMT = static_cast<VertexSE3 *>(optimizer.vertex(motionEdgeId));
      std::cerr << "the motion estimated: " << '\n';
      std::cerr << vMT->estimate().matrix() << '\n';

      // Set motion constraint
    }

} //namespace ORB_SLAM
