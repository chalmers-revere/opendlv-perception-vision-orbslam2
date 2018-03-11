/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
//#include "MapPoint.h"
//#include "Frame.h"

class PnPsolver {
 public:
  PnPsolver(/*const Frame &F, const std::vector<MapPoint*> &vpMapPointMatches*/);
  PnPsolver(PnPsolver const &) = delete;
  PnPsolver &operator=(PnPsolver const &) = delete;
  ~PnPsolver();

  void setRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4f,
                           float th2 = 5.991f);

  cv::Mat find(std::vector<bool> &inliers, int &nInliers);

  cv::Mat iterate(int nIterations, bool &stop, std::vector<bool> &inliers, int &nInliers);

 private:

  void CheckInliers();

  bool Refine();

  // Functions from the original EPnP code
  void setMaximumNumberOfCorrespondences(const int n);
  void resetCorrespondences(void);
  void addCorrespondence(const double X, const double Y, const double Z,
              const double u, const double v);

  double computePose(double R[3][3], double T[3]);

  void relativeError(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  void printPose(const double R[3][3], const double t[3]);
  double reprojectionError(const double R[3][3], const double t[3]);

  void chooseControlPoints(void);
  void computeBarycentricCoordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);


  double uc = 0, vc = 0, fu = 0, fv = 0;

  double * pws, * us, * alphas, * pcs;
  int m_maximumNumberOfCorrespondences;
  int number_of_correspondences;

  double cws[4][3], ccs[4][3];
  double cws_determinant = {};

  //std::vector<MapPoint*> mvpMapPointMatches;

  // 2D Points
  std::vector<cv::Point2f> mvP2D = {};
  std::vector<float> mvSigma2 ={};

  // 3D Points
  std::vector<cv::Point3f> mvP3Dw = {};

  // Index in Frame
  std::vector<size_t> mvKeyPointIndices = {};

  // Current Estimation
  double m_Ri[3][3];
  double m_ti[3];
  cv::Mat mTcwi= {};
  std::vector<bool> mvbInliersi= {};
  int mnInliersi= {};

  // Current Ransac State
  int m_nIterations;
  std::vector<bool> mvbBestInliers = {};
  int mnBestInliers;
  cv::Mat mBestTcw= {};

  // Refined
  cv::Mat mRefinedTcw = {};
  std::vector<bool> mvbRefinedInliers = {};
  int mnRefinedInliers= {};

  // Number of Correspondences
  int m_numberOfCorrespondences;

  // Indices for random selection [0 .. N-1]
  std::vector<size_t> m_allIndices = {};

  // RANSAC probability
  double m_ransacProb=0.99 ;

  // RANSAC min inliers
  int m_ransacMinInliers = 8;

  // RANSAC max iterations
  int m_ransacMaxIts = 300;

  // RANSAC Minimun Set used at each iteration
  int m_ransacMinSet = 4;

  // RANSAC expected inliers/total ratio
  float m_ransacEpsilon = 0.4f;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float m_ransacTh = 0.0f;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  std::vector<float> mvMaxError = {};

};

#endif //PNPSOLVER_H
