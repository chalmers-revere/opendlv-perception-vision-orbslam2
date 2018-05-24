/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* Modified for use within the OpenDLV framework by Marcus Andersson, Martin Baerveldt, Linus Eiderström Swahn and Pontus Pohl
* Copyright (C) 2018 Chalmers Revere
* For more information see <https://github.com/chalmers-revere/opendlv-perception-vision-orbslam2>
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


#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "orboptimizer.hpp"
#include "orbconverter.hpp"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> orbBlockSolver; 
typedef g2o::LinearSolverEigen<orbBlockSolver::PoseMatrixType> orbLinearSolver;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<7,3>> essentialBlockSolver;
typedef g2o::LinearSolverEigen<essentialBlockSolver::PoseMatrixType> essentialLinearSolver;

OrbOptimizer::OrbOptimizer()
{
}
OrbOptimizer::~OrbOptimizer()
{
}

void OrbOptimizer::GlobalBundleAdjustemnt(std::shared_ptr<OrbMap> pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    std::vector<std::shared_ptr<OrbKeyFrame>> vpKFs = pMap->GetAllKeyFrames();
    std::vector<std::shared_ptr<OrbMapPoint>> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}

void OrbOptimizer::BundleAdjustment(const std::vector<std::shared_ptr<OrbKeyFrame>> &vpKFs, const std::vector<std::shared_ptr<OrbMapPoint>> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{

    std::vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    auto linearSolver = g2o::make_unique<orbLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<orbBlockSolver>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Orbconverter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = static_cast<float>(std::sqrt(5.99));
    const float thHuber3D = static_cast<float>(std::sqrt(7.815));

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = vpMP[i];
        if(pMP->IsCorrupt())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Orbconverter::toVector3d(pMP->GetWorldPosition()));
        const int id = pMP->GetSequenceId()+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const std::map<std::shared_ptr<OrbKeyFrame>,size_t> observations = pMP->GetObservingKeyframes();

        int nEdges = 0;
        //SET EDGES
        for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            std::shared_ptr<OrbKeyFrame> pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            std::vector<cv::KeyPoint> vectorKP = pKF->mvKeysUn;
            const cv::KeyPoint &kpUn = vectorKP[mit->second];
            std::vector<float> vectorRight = pKF->mvuRight;
            if(vectorRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, optimizer.vertex(id));
                e->setVertex(1, optimizer.vertex(pKF->mnId));
                e->setMeasurement(obs);

                std::vector<float> invSigma2Vector = pKF->mvInvLevelSigma2;
                const float invSigma2 = invSigma2Vector[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = vectorRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);


                std::vector<float> invSigma2Vector = pKF->mvInvLevelSigma2;
                const float invSigma2 = invSigma2Vector[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
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

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Orbconverter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Orbconverter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        std::shared_ptr<OrbMapPoint> pMP = vpMP[i];

        if(pMP->IsCorrupt())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->GetSequenceId()+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
            pMP->UpdateMeanAndDepthValues();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Orbconverter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int OrbOptimizer::PoseOptimization(std::shared_ptr<OrbFrame> pFrame)
{
    
    g2o::SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<orbLinearSolver>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<orbBlockSolver>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;
    const int N = pFrame->N;
    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Orbconverter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    std::vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    std::vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    std::vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = static_cast<float>(std::sqrt(5.991));
    const float deltaStereo = static_cast<float>(std::sqrt(7.815));


    {
    //std::unique_lock<std::mutex> lock(OrbMapPoint::mGlobalMutex); //WHICH MUTEX??

    std::vector<float> vectorRight = pFrame->mvuRight;
    std::vector<cv::KeyPoint> vectorUndistortedKeyPoints = pFrame->mvKeysUn;
    std::vector<float> vectorInvLevelSigma2 = pFrame->mvInvLevelSigma2;
    std::vector<bool> vectorBoolKeyPoints = pFrame->mvbOutlier;
    for(int i=0; i<N; i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation

            if(vectorRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;

                const cv::KeyPoint &kpUn = vectorUndistortedKeyPoints[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = vectorInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPosition();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = vectorUndistortedKeyPoints[i];
                const float &kp_ur = vectorRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = vectorInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPosition();
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


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991f,5.991f,5.991f,5.991f};
    const float chi2Stereo[4]={7.815f,7.815f,7.815f, 7.815f};
    const int its[4]={10,10,10,10};    
    //std::cout << vpEdgesMono.size() << std::endl;
    int nBad=0;
    for(size_t it=0; it<4; it++)
    {
        std::vector<bool> vectorBoolOutliers = pFrame->mvbOutlier;
        vSE3->setEstimate(Orbconverter::toSE3Quat(pFrame->mTcw));
        //std::cout << "input pose: " << Orbconverter::toSE3Quat(pFrame->mTcw) << std::endl; 
        optimizer.initializeOptimization(0);
        //optimizer.setVerbose(true);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(vectorBoolOutliers[idx])
            {
                e->computeError();
            }

            const float chi2 = static_cast<float>(e->chi2());

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(vectorBoolOutliers[idx])
            {
                e->computeError();
            }

            const float chi2 = static_cast<float>(e->chi2());

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }    

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Orbconverter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

void OrbOptimizer::LocalBundleAdjustment(std::shared_ptr<OrbKeyFrame> pKF, bool* pbStopFlag, std::shared_ptr<OrbMap> pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    std::list<std::shared_ptr<OrbKeyFrame>> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    //pKF->mnBALocalForKF = pKF->Id;

    pKF->mnBALocalForKF = pKF->mnId;

    const std::vector<std::shared_ptr<OrbKeyFrame>> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    std::list<std::shared_ptr<OrbMapPoint>> lLocalMapPoints;
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::vector<std::shared_ptr<OrbMapPoint>> vpMPs = (*lit)->GetMapPointMatches();
        for(std::vector<std::shared_ptr<OrbMapPoint>>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            std::shared_ptr<OrbMapPoint> pMP = *vit;
            if(pMP)
                if(!pMP->IsCorrupt())
                    if(pMP->GetBALocalForKF()!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->SetBALocalForKF(pKF->mnId);
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    std::list<std::shared_ptr<OrbKeyFrame>> lFixedCameras;
    for(std::list<std::shared_ptr<OrbMapPoint>>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::map<std::shared_ptr<OrbKeyFrame>,size_t> observations = (*lit)->GetObservingKeyframes();
        for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            std::shared_ptr<OrbKeyFrame> pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<orbLinearSolver>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<orbBlockSolver>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Orbconverter::toSE3Quat(pKFi->GetPose()));
        //std::cout << "Pose: " << pKFi->GetPose() << std::endl;
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Orbconverter::toSE3Quat(pKFi->GetPose()));
        //std::cout << "Pose: " << pKFi->GetPose() << std::endl;
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbKeyFrame>> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbMapPoint>> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbKeyFrame>> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbMapPoint>> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = static_cast<float>(std::sqrt(5.991));
    const float thHuberStereo = static_cast<float>(std::sqrt(7.815));

    for(std::list<std::shared_ptr<OrbMapPoint>>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *lit;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Orbconverter::toVector3d(pMP->GetWorldPosition()));
        int id = pMP->GetSequenceId()+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const std::map<std::shared_ptr<OrbKeyFrame>,size_t> observations = pMP->GetObservingKeyframes();
                

        //Set edges
        for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            std::shared_ptr<OrbKeyFrame> pKFi = mit->first;
        	std::vector<float> invSigma2Vector = pKFi->mvInvLevelSigma2;
            std::vector<cv::KeyPoint> vectorKP = pKFi->mvKeysUn;
            std::vector<float> vectorRight = pKFi->mvuRight;



            if(!pKFi->isBad())
            {                
	
            const cv::KeyPoint &kpUn = vectorKP[mit->second];

                // Monocular observation
                if(vectorRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float invSigma2 = invSigma2Vector[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
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
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = vectorRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float invSigma2 = invSigma2Vector[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
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

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    //optimizer.setVerbose(true);
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        std::shared_ptr<OrbMapPoint> pMP = vpMapPointEdgeMono[i];

        if(pMP->IsCorrupt())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        std::shared_ptr<OrbMapPoint> pMP = vpMapPointEdgeStereo[i];

        if(pMP->IsCorrupt())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    std::vector<std::pair<std::shared_ptr<OrbKeyFrame>,std::shared_ptr<OrbMapPoint>> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        std::shared_ptr<OrbMapPoint> pMP = vpMapPointEdgeMono[i];

        if(pMP->IsCorrupt())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            std::shared_ptr<OrbKeyFrame> pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        std::shared_ptr<OrbMapPoint> pMP = vpMapPointEdgeStereo[i];

        if(pMP->IsCorrupt())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            std::shared_ptr<OrbKeyFrame> pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->m_MapUpdateMutex);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            std::shared_ptr<OrbKeyFrame> pKFi = vToErase[i].first;
            std::shared_ptr<OrbMapPoint> pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservingKeyframe(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbKeyFrame> pKFl = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFl->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKFl->SetPose(Orbconverter::toCvMat(SE3quat));
    }

    //Points
    for(std::list<std::shared_ptr<OrbMapPoint>>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->GetSequenceId()+maxKFid+1));
        pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
        pMP->UpdateMeanAndDepthValues();
    }
}

//OPTIMIZE GRAPH HERE
void OrbOptimizer::OptimizeEssentialGraph(std::shared_ptr<OrbMap> pMap, std::shared_ptr<OrbKeyFrame> pLoopKF, std::shared_ptr<OrbKeyFrame> pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const std::map<std::shared_ptr<OrbKeyFrame>, std::set<std::shared_ptr<OrbKeyFrame>> > &LoopConnections,
                                       const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<essentialLinearSolver>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<essentialBlockSolver>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const std::vector<std::shared_ptr<OrbKeyFrame>> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<std::shared_ptr<OrbMapPoint>> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->MaxKeyFrameId();

    std::vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    std::vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    std::vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Orbconverter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Orbconverter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    std::set<std::pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(std::map<std::shared_ptr<OrbKeyFrame>, std::set<std::shared_ptr<OrbKeyFrame>> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const std::set<std::shared_ptr<OrbKeyFrame>> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(std::set<std::shared_ptr<OrbKeyFrame>>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(std::make_pair(std::min(nIDi,nIDj),std::max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        std::shared_ptr<OrbKeyFrame> pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const std::set<std::shared_ptr<OrbKeyFrame>> sLoopEdges = pKF->GetLoopEdges();
        for(std::set<std::shared_ptr<OrbKeyFrame>>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            std::shared_ptr<OrbKeyFrame> pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const std::vector<std::shared_ptr<OrbKeyFrame>> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(std::vector<std::shared_ptr<OrbKeyFrame>>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            std::shared_ptr<OrbKeyFrame> pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(std::make_pair(std::min(pKF->mnId,pKFn->mnId),std::max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
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

    std::unique_lock<std::mutex> lock(pMap->m_MapUpdateMutex);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Orbconverter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = vpMPs[i];

        if(pMP->IsCorrupt())
            continue;

        int nIDr;
        if(pMP->GetCorrectedByKF()==pCurKF->mnId)
        {
            nIDr = pMP->GetCorrectedReference();
        }
        else
        {
            std::shared_ptr<OrbKeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPosition();
        Eigen::Matrix<double,3,1> eigP3Dw = Orbconverter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Orbconverter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPosition(cvCorrectedP3Dw);

        pMP->UpdateMeanAndDepthValues();
    }
}


int OrbOptimizer::OptimizeSim3(std::shared_ptr<OrbKeyFrame> pKF1, std::shared_ptr<OrbKeyFrame> pKF2, std::vector<std::shared_ptr<OrbMapPoint>> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<essentialLinearSolver>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<essentialBlockSolver>(std::move(linearSolver)));
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
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const std::vector<std::shared_ptr<OrbMapPoint>> vpMapPoints1 = pKF1->GetMapPointMatches();
    std::vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    std::vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    std::vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = static_cast<float>(std::sqrt(th2));

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        std::shared_ptr<OrbMapPoint> pMP1 = vpMapPoints1[i];
        std::shared_ptr<OrbMapPoint> pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetObeservationIndexOfKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->IsCorrupt() && !pMP2->IsCorrupt() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPosition();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Orbconverter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPosition();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Orbconverter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        std::vector<cv::KeyPoint> vectorKP1 = pKF1->mvKeysUn;
		std::vector<cv::KeyPoint> vectorKP2 = pKF2->mvKeysUn;
        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = vectorKP1[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, optimizer.vertex(id2));
        e12->setVertex(1, optimizer.vertex(0));
        e12->setMeasurement(obs1);


        std::vector<float> invSigma2Vector1 = pKF1->mvInvLevelSigma2;
        const float &invSigmaSquare1 = invSigma2Vector1[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = vectorKP2[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, optimizer.vertex(id1));
        e21->setVertex(1, optimizer.vertex(0));
        e21->setMeasurement(obs2);


        std::vector<float> invSigma2Vector2 = pKF2->mvInvLevelSigma2;
        float invSigmaSquare2 = invSigma2Vector2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
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
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();
    return nIn;
}



