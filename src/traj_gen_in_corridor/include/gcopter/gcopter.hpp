/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/solver/lbfgs.hpp"
#include "traj_gen_in_corridor/visualizer.hpp"
#include "ros/ros.h"
#include "quadrotor_msgs/OptCostDebug.h"
#include "gcopter/trajectory.hpp"
#include "gcopter/solver/flatness.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace gcopter
{

    class GCOPTER
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::Matrix<double, 6, -1> PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
#if TRAJ_ORDER == 3
        minco::MINCO_S2NU minco;
#elif TRAJ_ORDER == 5
        minco::MINCO_S3NU minco;
#elif TRAJ_ORDER == 7
        minco::MINCO_S4NU minco;
#endif

        bool isdebug;
        flatness::FlatnessMap flatmap;
        Visualizer *visualTraj;
        ros::Publisher *optPub;
        Trajectory<TRAJ_ORDER> Traj;
        quadrotor_msgs::OptCostDebug optCost;

        double rho;                           
        Eigen::Matrix<double, 3, 4> headPVAJ; 
        Eigen::Matrix<double, 3, 4> tailPVAJ; 

        PolyhedraV vPolytopes;      
        PolyhedraH hPolytopes;      
        Eigen::Matrix3Xd shortPath; 

        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;
        Eigen::VectorXi hPolyIdx;

        int polyN;       
        int pieceN;      
        int optCount;    
        int spatialDim;  
        int temporalDim; 

        double smoothEps;            
        int integralRes;             
        Eigen::VectorXi resVector;   
        Eigen::VectorXd magnitudeBd; 
        Eigen::VectorXd penaltyWt;   
        double allocSpeed;           

        double total_cost, minctrl_cost, time_cost;
        double pos_cost, vel_cost, acc_cost, omg_cost;

        lbfgs::lbfgs_parameter_t lbfgs_params; 

        Eigen::Vector3i totalOptCount;          
        Eigen::Matrix3Xd points;                
        Eigen::VectorXd times;                  
        // optional initial per-segment times provided by upstream (e.g., sample)
        Eigen::VectorXd initial_segment_times_;
        Eigen::Matrix3Xd gradByPoints;          
        Eigen::VectorXd gradByTimes;            
        Eigen::MatrixX3d partialGradByCoeffs;   
        Eigen::VectorXd partialGradByTimes;     

    private:
        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
        {
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                xi.segment(j, k).setConstant(sqrt(1.0 / k));
                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd xii = xi.segment(j, k);
                lbfgs::lbfgs_optimize(xii,
                                      minSqrD,
                                      &GCOPTER::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);
                xi.segment(j, k) = xii;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               double &cost,
                                               EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q = xi.segment(j, k);
                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        static inline void setResVector(const Eigen::Vector3d &start,
                                        const Eigen::Vector3d &end,
                                        const Eigen::Matrix3Xd &points,
                                        const Eigen::VectorXd &times,
                                        const double integralRes,
                                        Eigen::VectorXi &resVector)
        {
            int N = times.size();
            if (N <= 1) {
                // No interior sample points: set single resolution based on start-end distance
                double dis = (end - start).norm();
                resVector.resize(N);
                if (N == 1) resVector(0) = static_cast<int>(dis * integralRes + 1);
                return;
            }

            double dis = sqrt(pow(points(0, 0) - start(0), 2) + pow(points(1, 0) - start(1), 2) + pow(points(2, 0) - start(2), 2));
            resVector(0) = (int)(dis * integralRes + 1);
            for (int i = 1; i < N - 1; i++)
            {
                dis = sqrt(pow(points(0, i) - points(0, i - 1), 2) + pow(points(1, i) - points(1, i - 1), 2) + pow(points(2, i) - points(2, i - 1), 2));
                resVector(i) = (int)(dis * integralRes + 1);
            }
            dis = sqrt(pow(points(0, N - 2) - end(0), 2) + pow(points(1, N - 2) - end(1), 2) + pow(points(2, N - 2) - end(2), 2));
            resVector(N - 1) = (int)(dis * integralRes + 1);
        }

        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df)
        {
            if (x < 0.0)
            {
                return false;
            }
            else if (x > mu)
            {
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        static inline void cal_yaw(const Eigen::Vector3d &vel,
                                   const Eigen::Vector4d &quat,
                                   double &yaw_cal,
                                   double &yaw_dot_cal)
        {
            // calculate yaw
            Eigen::Vector3d zb, zb_norm, dir, dir_xb, xb, g;
            Eigen::Quaterniond q, q_, ori;
            Eigen::Matrix3d R;

            ori = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3));
            R = ori.normalized().toRotationMatrix();
            xb = Eigen::Vector3d(R(0, 0), R(1, 0), R(2, 0));
            zb = Eigen::Vector3d(R(0, 2), R(1, 2), R(2, 2));
            zb_norm = zb.normalized();

            dir = vel;
            dir_xb = dir - dir.dot(zb_norm) * zb_norm;

            double theta = acos(xb.dot(dir_xb.normalized()));
            if (zb.dot(xb.cross(dir_xb)) > 0)
                yaw_cal = theta;
            else
                yaw_cal = -theta;
        }

        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const Eigen::VectorXi &hIdx,
                                                   const PolyhedraH &hPolys,
                                                   const double &smoothFactor,
                                                   const Eigen::VectorXi &resolutionVector,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   const bool &isStartOptOmgZ,
                                                   const bool &isStartOptFlip,
                                                   flatness::FlatnessMap &flatmap,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC,
                                                   double &pos_cost,
                                                   double &vel_cost,
                                                   double &acc_cost,
                                                   double &omg_cost)
        {
            vel_cost = 0;
            acc_cost = 0;
            omg_cost = 0;
            const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double thrustMean = 0.5 * (magnitudeBounds(2) + magnitudeBounds(3));
            const double thrustRadi = 0.5 * fabs(magnitudeBounds(2) - magnitudeBounds(3));
            const double thrustSqrRadi = thrustRadi * thrustRadi;

            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            Eigen::Vector3d g(0, 0, 10);

            Eigen::Vector3d pos, vel, acc, jer, sna, dir, ddir;
            Eigen::Vector3d gradPos, gradVel, gradOmg;
            Eigen::Vector3d totalgradPos, totalgradVel, totalgradAcc, totalgradJer;
            Eigen::Vector3d totalgradDir, totalgradDirD;
            double totalgradpsi, totalgraddpsi;
            double gradThr;

            double step, alpha, thr, omg_z;
            double s1, s2, s3, s4, s5, s6, s7;
            Eigen::Matrix<double, TRAJ_ORDER + 1, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal, omg;
            Eigen::Vector4d quat;

            int K, L;
            double violaPos, violaThr, violaOmg;
            double violaPosPenaD, violaThrPenaD, violaOmgPenaD;
            double violaPosPena, violaThrPena, violaOmgPena;
            double node, pena;

            const int pieceNum = T.size();
            double integralFrac;
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, TRAJ_ORDER + 1, 3> &c = coeffs.block<TRAJ_ORDER + 1, 3>(i * (TRAJ_ORDER + 1), 0);
                integralFrac = 1.0 / resolutionVector(i);
                step = T(i) * integralFrac;
                for (int j = 0; j <= resolutionVector(i); j++)
                {
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    s6 = s3 * s3;
                    s7 = s6 * s1;
#if TRAJ_ORDER == 3
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0;
#elif TRAJ_ORDER == 5
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
#elif TRAJ_ORDER == 7
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
#endif
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;
                    dir = vel;
                    ddir = acc;

                    flatmap.forward(vel, acc, jer, 0, 0, thr, quat, omg);
                    omg_z = omg(2);
                    omg(2) = 0;
                    
                    // 总范数约束：限制速度和加速度的欧氏范数
                    const double velMax = magnitudeBounds(0);
                    const double accMax = magnitudeBounds(1);
                    // 使用外层已声明的变量：omgSqrMax, thrustMean, thrustSqrRadi
                    
                    // 总范数速度/加速度约束
                    const double velNorm = vel.norm();
                    const double accNorm = acc.norm();
                    const double violaVel = velNorm - velMax;
                    const double violaAcc = accNorm - accMax;
                    
                    // 角速度约束（保持原逻辑，使用外层声明的变量）
                    violaOmg = omg.squaredNorm() - omgSqrMax;
                    violaThr = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    Eigen::Vector3d gradAcc = Eigen::Vector3d::Zero();
                    pena = 0.0, gradThr = 0.0;
                    node = (j == 0 || j == resolutionVector(i)) ? 0.5 : 1.0;

                    L = hIdx(i);
                    K = hPolys[L].cols();
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].col(k).head<3>();
                        violaPos = outerNormal.dot(pos - hPolys[L].col(k).tail<3>());
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos += weightPos * violaPosPenaD * outerNormal;
                            pos_cost += node * step * weightPos * violaPosPena;
                            pena += weightPos * violaPosPena;
                        }
                    }

                    // 总范数速度约束处理
                    double violaVelPenaD, violaVelPena;
                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        if (velNorm > 1e-9)
                        {
                            gradVel += weightVel * violaVelPenaD * (vel / velNorm);
                        }
                        vel_cost += node * step * weightVel * violaVelPena;
                        pena += weightVel * violaVelPena;
                    }

                    // 总范数加速度约束处理
                    const double weightAcc = penaltyWeights(7);  // 新增加速度权重
                    double violaAccPenaD, violaAccPena;
                    if (smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD))
                    {
                        if (accNorm > 1e-9)
                        {
                            gradAcc += weightAcc * violaAccPenaD * (acc / accNorm);
                        }
                        acc_cost += node * step * weightAcc * violaAccPena;
                        pena += weightAcc * violaAccPena;
                    }

                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                    {
                        gradOmg += penaltyWeights(2) * violaOmgPenaD * 2.0 * omg;
                        omg_cost += node * step * penaltyWeights(2) * violaOmgPena;
                        pena += penaltyWeights(2) * violaOmgPena;
                    }

                    if (smoothedL1(violaThr, smoothFactor, violaThrPena, violaThrPenaD))
                    {
                        gradThr += penaltyWeights(3) * violaThrPenaD * 2.0 * (thr - thrustMean);
                        acc_cost += node * step * penaltyWeights(3) * violaThrPena;
                        pena += penaltyWeights(3) * violaThrPena;
                    }

                    flatmap.backward(gradPos, gradVel, gradThr, Eigen::Vector4d::Zero(), gradOmg,
                                     totalgradPos, totalgradVel, totalgradAcc, totalgradJer, totalgradpsi, totalgraddpsi);
                    
                    // 加上加速度梯度
                    totalgradAcc += gradAcc;

                    alpha = j * integralFrac;
                    gradC.block(i * (TRAJ_ORDER + 1), 0, TRAJ_ORDER + 1, 3) += (beta0 * totalgradPos.transpose() +
                                                                                beta1 * totalgradVel.transpose() +
                                                                                beta2 * totalgradAcc.transpose() +
                                                                                beta3 * totalgradJer.transpose()) *
                                                                               node * step;
                    gradT(i) += (totalgradPos.dot(vel) +
                                 totalgradVel.dot(acc) +
                                 totalgradAcc.dot(jer) +
                                 totalgradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    cost += node * step * pena;
                }
            }
            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &grad)
        {
            GCOPTER &obj = *(GCOPTER *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(grad.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(grad.data() + dimTau, dimXi);

            forwardT(tau, obj.times);
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

            double cost = 0;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minctrl_cost = cost;
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
            cost *= obj.penaltyWt(6);
            obj.minctrl_cost = cost;
            obj.partialGradByCoeffs *= obj.penaltyWt(6);
            obj.partialGradByTimes *= obj.penaltyWt(6);

            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.hPolytopes,
                                    obj.smoothEps, obj.resVector, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, false,
                                    false, obj.flatmap, cost,
                                    obj.partialGradByTimes, obj.partialGradByCoeffs,
                                    obj.pos_cost, obj.vel_cost, obj.acc_cost, obj.omg_cost);

            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);

            obj.time_cost = weightT * obj.times.sum();
            cost += obj.time_cost;
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            obj.total_cost = cost;
            return cost;
        }

        static inline int costProgress(void *ptr,
                                       const Eigen::VectorXd &x,
                                       const Eigen::VectorXd &grad,
                                       const double fx,
                                       const double step,
                                       const int k,
                                       const int ls)
        {
            GCOPTER &obj = *(GCOPTER *)ptr;
            obj.optCount = k;

            if (obj.isdebug)
            {
                // publish msg
                obj.optCost.iter = k;
                obj.optCost.line_search_count = ls;
                obj.optCost.gnorm = grad.norm();
                obj.optCost.total_cost = fx;
                obj.optCost.ctrl_cost = obj.minctrl_cost;
                obj.optCost.time_cost = obj.time_cost;
                obj.optCost.pos_cost = obj.pos_cost;
                obj.optCost.vel_cost = obj.vel_cost;
                obj.optCost.acc_cost = obj.acc_cost;
                obj.optCost.omg_cost = obj.omg_cost;
                obj.optCost.flip_cost_sum = 0.0;
                obj.optCost.flip_pos_cost_sum = 0.0;
                obj.optPub->publish(obj.optCost);
            }

            if (obj.isdebug && (k % 50 == 0 || k == 1))
            {
                const int dimTau = obj.temporalDim, dimXi = obj.spatialDim;
                forwardT(x.head(dimTau), obj.times);
                forwardP(x.tail(dimXi), obj.vPolyIdx, obj.vPolytopes, obj.points);

                obj.minco.getTrajectory(obj.Traj);
                obj.visualTraj->visualize(obj.Traj, 0);
            }

            return 0;
        }

        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &x,
                                          Eigen::VectorXd &grad)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(x.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(x.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(grad.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;

            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }

            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;

            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GCOPTER::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);

                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        static inline void sortCorridor(PolyhedronV &vpoly)
        {
            int minidx, n = vpoly.cols();
            double minpos, temppos;
            for (int i = 0; i < n; i++)
            {
                minidx = i;
                minpos = vpoly.col(i).sum();
                for (int j = i + 1; j < n; j++)
                {
                    temppos = vpoly.col(j).sum();
                    if (minpos > temppos)
                    {
                        minidx = j;
                        minpos = temppos;
                    }
                }
                vpoly.col(i).swap(vpoly.col(minidx));
            }
        }

        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {

                if (!geoutils::enumerateVs(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                sortCorridor(curIV);
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
                curIH.leftCols(hPs[i].cols()) = hPs[i];
                curIH.rightCols(hPs[i + 1].cols()) = hPs[i + 1];

                // Inflate the intersection half-spaces slightly to create a larger
                // overlap region for the optimizer. This gives MINCO more freedom
                // at the connection point and avoids forcing unnatural S-shaped
                // corrections when the true physical overlap is tiny.
                const double overlap_inflate = 0.30; // meters
                for (int jj = 0; jj < curIH.cols(); ++jj) {
                    Eigen::Vector3d nor = curIH.col(jj).head<3>();
                    double nrm = nor.norm();
                    if (nrm > 1e-9) {
                        // move the supporting point along the normal by inflate distance
                        curIH.col(jj).tail<3>() += (nor / nrm) * overlap_inflate;
                    }
                }

                if (!geoutils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                sortCorridor(curIV);
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geoutils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            sortCorridor(curIV);
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            innerPoints.resize(3, sizeN - 1);
            timeAlloc.resize(sizeN);

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);
                b = path.col(i + 1);
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

    public:
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                          const Eigen::Matrix<double, 3, 4> &terminalPVAJ,
                          const PolyhedraH &safeCorridor,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const int &flipResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &useKeyPos,
                          const Eigen::Matrix3Xd &attSequence,
                          const Eigen::Matrix3Xd &attPose,
                          const Eigen::VectorXd &attTimeProportion,
                          const bool &isDebug,
                          const bool &isOptSetTime,
                          Visualizer &visual,
                          ros::Publisher &optPuber,
                          const Eigen::VectorXd &initial_segment_times = Eigen::VectorXd())
        {
            isdebug = isDebug;
            visualTraj = &visual;
            optPub = &optPuber;
            rho = timeWeight;
            headPVAJ = initialPVAJ;
            tailPVAJ = terminalPVAJ;

            hPolytopes = safeCorridor;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            { 
                hPolytopes[i].topRows<3>().colwise().normalize();
            }
            
            bool corridor_failed;
            corridor_failed = !processCorridor(hPolytopes, vPolytopes);
            if (corridor_failed)
            {
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl
                          << "                Corridor process failed!" << std::endl
                          << "      Press ENTER to check which intention is wrong!" << std::endl
                          << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                return false;
            }

            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            allocSpeed = magnitudeBd(0);

            // store optional initial segment times for use in optimize()
            initial_segment_times_ = initial_segment_times;

            getShortestPath(headPVAJ.col(0), tailPVAJ.col(0),
                            vPolytopes, smoothEps, shortPath);

            const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
            pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            pieceIdx.array() += 1;
            pieceN = pieceIdx.sum();

            temporalDim = pieceN;
            spatialDim = 0;
            vPolyIdx.resize(pieceN - 1);
            hPolyIdx.resize(pieceN);
            resVector.resize(pieceN);

            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {                 
                    if (l < k - 1) 
                    {              
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1) 
                    {
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    hPolyIdx(j) = i;
                }
            }

#if TRAJ_ORDER == 3
            minco.setConditions(headPVAJ.leftCols(2), tailPVAJ.leftCols(2), pieceN);
#elif TRAJ_ORDER == 5
            minco.setConditions(headPVAJ.leftCols(3), tailPVAJ.leftCols(3), pieceN);
#elif TRAJ_ORDER == 7
            minco.setConditions(headPVAJ, tailPVAJ, pieceN);
#else
            return false;
#endif

            // Allocate temp variables
            points.resize(3, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(3, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize((TRAJ_ORDER + 1) * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        inline double optimize(Trajectory<TRAJ_ORDER> &traj,
                               const double &relCostTol)
        {
            Traj = traj;
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);

            // If an initial per-segment time vector was provided (e.g., from sample),
            // map it to piece-level times. If the provided vector length equals
            // the piece count, use it directly. Otherwise, distribute the total
            // provided time proportionally to piece geometric lengths.
            if (initial_segment_times_.size() > 0) {
                if (initial_segment_times_.size() == pieceN) {
                    times = initial_segment_times_;
                    ROS_INFO("GCOPTER: using provided per-piece initial times (size==pieceN)");
                } else {
                    // compute per-piece geometric lengths
                    Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
                    Eigen::VectorXd pieceLengths(pieceN);
                    int idx = 0;
                    for (int i = 0; i < polyN; ++i) {
                        double seg_len = deltas.col(i).norm();
                        int k = pieceIdx(i);
                        if (k <= 0) k = 1;
                        for (int j = 0; j < k; ++j) {
                            pieceLengths(idx++) = seg_len / double(k);
                        }
                    }
                    double provided_total = initial_segment_times_.sum();
                    if (provided_total > 1e-9) {
                        double len_sum = pieceLengths.sum();
                        if (len_sum <= 1e-9) len_sum = 1.0;
                        times = provided_total * pieceLengths / len_sum;
                        ROS_INFO("GCOPTER: distributed provided total time %.3f to pieces (len_sum=%.3f)", provided_total, len_sum);
                    } else {
                        ROS_WARN("GCOPTER: provided initial_segment_times sum is zero, keeping default times");
                    }
                }
            }
            
            // set sample point number
            setResVector(headPVAJ.col(0), tailPVAJ.col(0), points, times, integralRes, resVector);
            
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }
    };

} // namespace gcopter

#endif
