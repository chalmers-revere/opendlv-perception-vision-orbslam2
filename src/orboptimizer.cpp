/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include "orboptimizer.hpp"
#include "orbconverter.hpp"
/*typedef MatrixXd PoseMatrixType;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> > orbBlockSolver;
typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> orbLinearSolver;
typedef BlockSolver< BlockSolverTraits<6, 3> > BlockSolver_6_3;*/

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> orbBlockSolver; 
typedef g2o::LinearSolverEigen<orbBlockSolver::PoseMatrixType> orbLinearSolver;

OrbOptimizer::OrbOptimizer()
{
}
OrbOptimizer::~OrbOptimizer()
{
}

void OrbOptimizer::GlobalBundleAdjustemnt(std::shared_ptr<OrbMap> pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    std::vector<std::shared_ptr<OrbFrame>> vpKFs = pMap->GetAllKeyFrames();
    std::vector<std::shared_ptr<OrbMapPoint>> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}

void OrbOptimizer::BundleAdjustment(const std::vector<std::shared_ptr<OrbFrame>> &vpKFs, const std::vector<std::shared_ptr<OrbMapPoint>> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{

    std::vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    auto linearSolver = g2o::make_unique<orbLinearSolver>();

    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<orbBlockSolver>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        std::shared_ptr<OrbFrame> pKF = vpKFs[i];
        if(pKF->IsCorrupt())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Orbconverter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->Id);
        vSE3->setFixed(pKF->Id==0);
        optimizer.addVertex(vSE3);
        if(pKF->Id>maxKFid)
            maxKFid=pKF->Id;
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
        const int id = pMP->Id+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const std::map<std::shared_ptr<OrbFrame>,size_t> observations = pMP->GetObservingKeyframes();

        int nEdges = 0;
        //SET EDGES
        for(std::map<std::shared_ptr<OrbFrame>,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            std::shared_ptr<OrbFrame> pKF = mit->first;
            if(pKF->IsCorrupt() || pKF->Id>maxKFid)
                continue;

            nEdges++;

            std::vector<cv::KeyPoint> vectorKP = pKF->GetUndistortedKeyPoints();
            const cv::KeyPoint &kpUn = vectorKP[mit->second];
            std::vector<float> vectorRight = pKF->GetRight();
            if(vectorRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->Id)));
                e->setMeasurement(obs);

                std::vector<float> invSigma2Vector = pKF->GetInverseLevelSigma2();
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
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->Id)));
                e->setMeasurement(obs);


                std::vector<float> invSigma2Vector = pKF->GetInverseLevelSigma2();
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
        std::shared_ptr<OrbFrame> pKF = vpKFs[i];
        if(pKF->IsCorrupt())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->Id));
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
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->Id+maxKFid+1));

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

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Orbconverter::toSE3Quat(pFrame->GetPose()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->GetNumberOfKeyPoints();

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
    std::unique_lock<std::mutex> lock(OrbMapPoint::mGlobalMutex); //WHICH MUTEX??

    std::vector<float> vectorRight = pFrame->GetRight();
    std::vector<cv::KeyPoint> vectorUndistortedKeyPoints = pFrame->GetUndistortedKeyPoints();
    std::vector<float> vectorInvLevelSigma2 = pFrame->GetInverseLevelSigma2();
    std::vector<bool> vectorBoolKeyPoints = pFrame->GetBoolOutliers();
    for(int i=0; i<N; i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = pFrame->GetMapPoint(i);
        if(pMP)
        {
            // Monocular observation

            if(vectorRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->SetBoolOutliers(false,i);

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
                pFrame->SetBoolOutliers(false,i);

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

    int nBad=0;
    std::vector<bool> vectorBoolOutliers = pFrame->GetBoolOutliers();
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Orbconverter::toSE3Quat(pFrame->GetPose()));
        optimizer.initializeOptimization(0);
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
                pFrame->SetBoolOutliers(true,idx);
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->SetBoolOutliers(false,idx);
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
                pFrame->SetBoolOutliers(true,idx);
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->SetBoolOutliers(false,idx);
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

void OrbOptimizer::LocalBundleAdjustment(std::shared_ptr<OrbFrame> pKF, bool* pbStopFlag, std::shared_ptr<OrbMap> pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    std::list<std::shared_ptr<OrbFrame>> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    //pKF->mnBALocalForKF = pKF->Id;

    pKF->SetBALocalForKF(pKF->Id);

    const std::vector<std::shared_ptr<OrbFrame>> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        std::shared_ptr<OrbFrame> pKFi = vNeighKFs[i];
        pKFi->SetBALocalForKF(pKF->Id);;
        if(!pKFi->IsCorrupt())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    std::list<std::shared_ptr<OrbMapPoint>> lLocalMapPoints;
    for(std::list<std::shared_ptr<OrbFrame>>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::vector<std::shared_ptr<OrbMapPoint>> vpMPs = (*lit)->GetMapPointMatches();
        for(std::vector<std::shared_ptr<OrbMapPoint>>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            std::shared_ptr<OrbMapPoint> pMP = *vit;
            if(pMP)
                if(!pMP->IsCorrupt())
                    if(pMP->GetBALocalForKF()!=pKF->Id)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->SetBALocalForKF(pKF->Id);
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    std::list<std::shared_ptr<OrbFrame>> lFixedCameras;
    for(std::list<std::shared_ptr<OrbMapPoint>>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::map<std::shared_ptr<OrbFrame>,size_t> observations = (*lit)->GetObservingKeyframes();
        for(std::map<std::shared_ptr<OrbFrame>,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            std::shared_ptr<OrbFrame> pKFi = mit->first;

            if(pKFi->GetBALocalForKF()!=pKF->Id && pKFi->GetBAFixedForKF()!=pKF->Id)
            {                
                pKFi->SetBAFixedForKF(pKF->Id);
                if(!pKFi->IsCorrupt())
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
    for(std::list<std::shared_ptr<OrbFrame>>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbFrame> pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Orbconverter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->Id);
        vSE3->setFixed(pKFi->Id==0);
        optimizer.addVertex(vSE3);
        if(pKFi->Id>maxKFid)
            maxKFid=pKFi->Id;
    }

    // Set Fixed KeyFrame vertices
    for(std::list<std::shared_ptr<OrbFrame>>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbFrame> pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Orbconverter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->Id);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->Id>maxKFid)
            maxKFid=pKFi->Id;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbFrame>> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbMapPoint>> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    std::vector<std::shared_ptr<OrbFrame>> vpEdgeKFStereo;
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
        int id = pMP->Id+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const std::map<std::shared_ptr<OrbFrame>,size_t> observations = pMP->GetObservingKeyframes();
        std::vector<cv::KeyPoint> vectorKP = pKF->GetUndistortedKeyPoints();
        std::vector<float> vectorRight = pKF->GetRight();
                

        //Set edges
        for(std::map<std::shared_ptr<OrbFrame>,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            std::shared_ptr<OrbFrame> pKFi = mit->first;
        	std::vector<float> invSigma2Vector = pKFi->GetInverseLevelSigma2();


            if(!pKFi->IsCorrupt())
            {                
	
            const cv::KeyPoint &kpUn = vectorKP[mit->second];

                // Monocular observation
                if(vectorRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->Id)));
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
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->Id)));
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

    std::vector<std::pair<std::shared_ptr<OrbFrame>,std::shared_ptr<OrbMapPoint>> > vToErase;
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
            std::shared_ptr<OrbFrame> pKFi = vpEdgeKFMono[i];
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
            std::shared_ptr<OrbFrame> pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->m_MapUpdateMutex);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            std::shared_ptr<OrbFrame> pKFi = vToErase[i].first;
            std::shared_ptr<OrbMapPoint> pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservingKeyframe(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(std::list<std::shared_ptr<OrbFrame>>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbFrame> pKFl = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFl->Id));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKFl->SetPose(Orbconverter::toCvMat(SE3quat));
    }

    //Points
    for(std::list<std::shared_ptr<OrbMapPoint>>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->Id+maxKFid+1));
        pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
        pMP->UpdateMeanAndDepthValues();
    }
}
