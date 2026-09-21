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

#ifndef COLLISION_GCOPTER_HPP
#define COLLISION_GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/solver/lbfgs.hpp"
#include "traj_gen_in_corridor/visualizer.hpp"
#include "ros/ros.h"
#include "quadrotor_msgs/OptCostDebug.h"
#include "gcopter/trajectory.hpp"
#include "gcopter/solver/flatness.hpp"
#include "trt_manager.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace collision_gcopter
{
    // 碰撞事件结构体 - 用于存储从前端获得的真实碰撞信息
    struct CollisionEvent
    {
        Eigen::Vector3d collision_point;        // 碰撞点位置（固定参考）
        Eigen::Vector3d plane_normal;           // 碰撞平面法向量
        Eigen::Vector3d pre_collision_velocity; // 前端提供的碰撞前速度
        Eigen::Vector3d post_collision_velocity;// 前端提供的碰撞后速度
        double collision_time;                  // 碰撞发生时间
        double plane_d;                         // 平面方程参数d
        
        // 碰撞前约束参数（主要约束目标）
        Eigen::Vector3d reference_point;        // 参考点p_0（碰撞点）
        Eigen::Matrix3d rotation_matrix;        // 旋转矩阵R（将平面法向量转换为z轴）
        Eigen::Matrix<double, 3, 2> basis_matrix; // 基矩阵B（平面内的两个正交基向量）
        double max_position_offset;             // 最大位置偏移r（碰撞前）
        double given_yaw;                        // 前端给定的yaw（用于flatness/模型输入）
        
        CollisionEvent() : collision_point(Eigen::Vector3d::Zero()),
                          plane_normal(Eigen::Vector3d::Zero()),
                          pre_collision_velocity(Eigen::Vector3d::Zero()),
                          post_collision_velocity(Eigen::Vector3d::Zero()),
                          collision_time(0.0), plane_d(0.0),
                          reference_point(Eigen::Vector3d::Zero()),
                          rotation_matrix(Eigen::Matrix3d::Identity()),
                          basis_matrix(Eigen::Matrix<double, 3, 2>::Zero()),
                          max_position_offset(0.3),
                          given_yaw(0.0) {}
                          
        // 初始化碰撞前约束参数
        void initializeConstraintParameters()
        {
            // 基本数值检查
            if (!collision_point.allFinite() || !plane_normal.allFinite() || 
                !pre_collision_velocity.allFinite() || !post_collision_velocity.allFinite()) {
                return;
            }
            
            if (plane_normal.norm() < 1e-6 || !std::isfinite(collision_time)) {
                return;
            }
            
            // 设置参考点为碰撞点
            reference_point = collision_point;
            
            // 构建旋转矩阵，将平面法向量转换为z轴
            Eigen::Vector3d n = plane_normal.normalized();
            Eigen::Vector3d x_axis(1, 0, 0);
            
            if (std::abs(n.dot(x_axis)) > 0.9999) {
                rotation_matrix = Eigen::Matrix3d::Identity();
            } else {
                Eigen::Vector3d v = n.cross(x_axis);
                double s = v.norm();
                double c = n.dot(x_axis);

                if (s < 1e-6) {
                    rotation_matrix = Eigen::Matrix3d::Identity();
                } else {
                    Eigen::Matrix3d vx;
                    vx << 0, -v.z(), v.y(),
                          v.z(), 0, -v.x(),
                          -v.y(), v.x(), 0;
                    
                    rotation_matrix = Eigen::Matrix3d::Identity() + vx + vx * vx * ((1 - c) / (s * s));
                    
                    if (!rotation_matrix.allFinite()) {
                        rotation_matrix = Eigen::Matrix3d::Identity();
                    }
                }
            }
            
            // 构建平面内的正交基矩阵B
            Eigen::Vector3d u1, u2;
            if (std::abs(n.x()) < 0.9) {
                u1 = Eigen::Vector3d(1, 0, 0) - n.dot(Eigen::Vector3d(1, 0, 0)) * n;
            } else {
                u1 = Eigen::Vector3d(0, 1, 0) - n.dot(Eigen::Vector3d(0, 1, 0)) * n;
            }
            
            if (u1.norm() < 1e-6) {
                basis_matrix.setZero();
                return;
            }
            
            u1.normalize();
            u2 = n.cross(u1);
            
            if (u2.norm() < 1e-6) {
                basis_matrix.setZero();
                return;
            }
            
            u2.normalize();
            
            basis_matrix.col(0) = u1;
            basis_matrix.col(1) = u2;
            
            if (!basis_matrix.allFinite()) {
                basis_matrix.setZero();
                return;
            }
        }
        
        // 新增：动态计算碰撞后速度（基于当前轨迹状态），使用 TensorRT bs=1 碰撞模型
        // 线速度由 current_traj 提供，姿态和角速度通过 flatness 计算，yaw 由速度方向给定
        Eigen::Vector3d calculateDynamicPostCollisionVelocity(
            const Trajectory<TRAJ_ORDER>& current_traj,
            double collision_time = -1.0,
            double friction = 0.65,
            double damping_ratio = 0.10,
            double given_yaw = 0.0) const
        {
            // 如果碰撞时间为 -1，表示碰撞在段末端
            double actual_collision_time = collision_time;
            ROS_INFO_STREAM("actual_collision_time: " << actual_collision_time);
            double total_T = current_traj.getTotalDuration();
            if (collision_time < 0.0)
            {
                actual_collision_time = total_T;
            }

            // 确保时间在有效范围内
            if (actual_collision_time < 0.0 || actual_collision_time > total_T)
            {
                actual_collision_time = total_T;
            }

            // 碰撞前线速度 / 加速度 / jerk（世界坐标系）
            Eigen::Vector3d vel = current_traj.getVel(actual_collision_time);
            Eigen::Vector3d acc = current_traj.getAcc(actual_collision_time);
            Eigen::Vector3d jer = current_traj.getJer(actual_collision_time);

            // 基础数值检查
            if (!vel.allFinite() || !acc.allFinite() || !jer.allFinite())
            {
                return Eigen::Vector3d::Zero();
            }

            // 使用 flatness 计算姿态和角速度
            // 按用户要求：直接使用前端给定的 yaw，不做基于速度的变换；dpsi 固定为 0
            double psi = given_yaw;
            double dpsi = 0.0;
            double thr = 0.0;
            Eigen::Vector4d quat; // (w, x, y, z)
            Eigen::Vector3d omg;  // 机体系角速度

            flatness::FlatnessMap fm = flatness::FlatnessMap();
            fm.forward(vel, acc, jer, psi, dpsi, thr, quat, omg);

            if (!quat.allFinite() || !omg.allFinite())
            {
                return Eigen::Vector3d::Zero();
            }

            // 使用 TensorRT bs=1 碰撞模型
            Eigen::Vector3d plane_n = plane_normal;
            if (!plane_n.allFinite() || plane_n.norm() < 1e-6)
            {
                // 若法向异常，直接返回原速度
                return vel;
            }
            plane_n.normalize();

            // 分解速度为法向与切向分量（世界坐标系）
            double v_n_scalar = vel.dot(plane_n);
            Eigen::Vector3d v_t_vec = vel - v_n_scalar * plane_n;
            double v_t_norm = v_t_vec.norm();
            Eigen::Vector3d t_dir = Eigen::Vector3d::Zero();
            if (v_t_norm > 1e-6)
            {
                t_dir = v_t_vec / v_t_norm;
            }
            else
            {
                // 若切向速度极小，选取任一与法向正交的单位向量作为切向方向
                Eigen::Vector3d up(0.0, 0.0, 1.0);
                if (std::abs(plane_n.dot(up)) > 0.9)
                {
                    up = Eigen::Vector3d(0.0, 1.0, 0.0);
                }
                t_dir = (up - up.dot(plane_n) * plane_n).normalized();
                v_t_norm = 0.0;
            }

            try
            {
                trt::TrtManager &mgr = trt::TrtManager::instance();
                if (!mgr.isReady())
                {
                    ROS_ERROR("calculateDynamicPostCollisionVelocity: TRT manager not ready, no analytic fallback");
                    return Eigen::Vector3d::Zero();
                }

                CollisionPredictor &pred = mgr.collisionBs1();

                // -----------------------------------------------------------
                // 1. 速度分解 (保持原逻辑)
                // -----------------------------------------------------------
                // v_n_algebraic: 法向速度投影 (撞击时应为负)
                double v_n_algebraic = vel.dot(plane_n);
                
                // approach_speed: 撞击速率 (正值)
                // 如果正在远离墙壁 (>0)，在物理上不应发生碰撞，这里做个保护，设为0或按需处理
                double approach_speed = (v_n_algebraic < 0.0) ? -v_n_algebraic : 0.0;
                
                // 计算切向速度大小
                Eigen::Vector3d v_t_vec = vel - v_n_algebraic * plane_n;
                double v_t_norm = v_t_vec.norm();

                // 计算切向方向 (用于输出重建)
                Eigen::Vector3d t_dir = Eigen::Vector3d::Zero();
                if (v_t_norm > 1e-6) {
                    t_dir = v_t_vec / v_t_norm;
                } else {
                    // 处理垂直撞击的奇异点
                    Eigen::Vector3d up(0.0, 0.0, 1.0);
                    if (std::abs(plane_n.dot(up)) > 0.9) up = Eigen::Vector3d(0.0, 1.0, 0.0);
                    t_dir = (up - up.dot(plane_n) * plane_n).normalized();
                    v_t_norm = 0.0;
                }

                // -----------------------------------------------------------
                // 2. 正飞/倒飞 检测 (新增逻辑)
                // -----------------------------------------------------------
                // 恢复当前真实姿态
                Eigen::Quaterniond q_curr(quat(0), quat(1), quat(2), quat(3));
                
                // 将速度投影到机体坐标系
                Eigen::Vector3d v_body = q_curr.inverse() * vel;
                
                // 判断是否为机尾撞击: 机体X轴速度为负
                bool is_tail_collision = false;
                if (v_body.x() < -0.1) {
                    is_tail_collision = true;
                }

                // -----------------------------------------------------------
                // 3. 四元数变换 (核心修改)
                // -----------------------------------------------------------
                // A. 内修 (Body Flip): 如果是倒飞，绕机体Z轴转180度，伪装成正飞
                Eigen::Quaterniond q_flip = Eigen::Quaterniond::Identity();
                if (is_tail_collision) {
                    // 绕 Z 轴旋转 180 度
                    q_flip = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
                }

                // B. 外修 (Environment Alignment): 将真实墙面对齐到训练墙面 (-1, 0, 0)
                Eigen::Vector3d train_plane_normal(-1.0, 0.0, 0.0);
                Eigen::Quaterniond q_align = Eigen::Quaterniond::Identity();
                
                // 使用 Eigen 内置函数计算最短弧旋转，替代原本繁琐的 Rodrigues 计算
                q_align.setFromTwoVectors(plane_n, train_plane_normal);

                // C. 合成最终输入姿态
                // 公式: q_input = q_align(环境) * q_curr(当前) * q_flip(机身修正)
                Eigen::Quaterniond q_train = q_align * q_curr * q_flip;

                // -----------------------------------------------------------
                // 4. 角速度修正 (新增逻辑)
                // -----------------------------------------------------------
                // 如果机体翻转了 180 度 (q_flip)，机体坐标系下的角速度分量定义也会改变
                // 绕 Z 轴转 180 度矩阵为 diag(-1, -1, 1)
                // 所以 wx -> -wx, wy -> -wy, wz -> wz
                float in_wx = static_cast<float>(omg(0));
                float in_wy = static_cast<float>(omg(1));
                float in_wz = static_cast<float>(omg(2));

                if (is_tail_collision) {
                    in_wx = -in_wx;
                    in_wy = -in_wy;
                }

                // -----------------------------------------------------------
                // 5. 构造输入向量
                // -----------------------------------------------------------
                std::vector<float> input(12, 0.0f);
                
                input[0] = static_cast<float>(approach_speed); // 标量，无方向，无需旋转
                input[1] = static_cast<float>(v_t_norm);       // 标量，无方向，无需旋转
                
                // 这里的 vel.z() 是世界坐标系下的 Z 速度。
                // 如果你的模型假设 Z 轴绝对垂直(重力方向)，则不受 q_align (水平旋转) 影响。
                // 如果 q_align 包含 Pitch/Roll 旋转，这里可能需要注意，但通常暂且保持原样。
                input[2] = static_cast<float>(vel.z()); 
                
                input[3] = in_wx;
                input[4] = in_wy;
                input[5] = in_wz;

                // 输入处理后的虚拟姿态
                input[6] = static_cast<float>(q_train.w());
                input[7] = static_cast<float>(q_train.x());
                input[8] = static_cast<float>(q_train.y());
                input[9] = static_cast<float>(q_train.z());

                input[10] = static_cast<float>(friction);
                input[11] = static_cast<float>(damping_ratio);

                // 日志打印 (可选)
                // ROS_INFO("Collision Input (Tail=%d): spd=%.2f, q=(%.2f, %.2f, %.2f, %.2f)", 
                //          is_tail_collision, approach_speed, q_train.w(), q_train.x(), q_train.y(), q_train.z());

                // -----------------------------------------------------------
                // 6. 黑盒预测
                // -----------------------------------------------------------
                std::vector<float> output;
                pred.predict(input, output, 1);
                if (output.size() < 2)
                {
                    ROS_ERROR("calculateDynamicPostCollisionVelocity: TRT produced incomplete output");
                    return Eigen::Vector3d::Zero();
                }

                double post_n_scalar = static_cast<double>(output[0]);
                double post_t_scalar = static_cast<double>(output[1]);

                // -----------------------------------------------------------
                // 7. 输出重建 (保持鲁棒性逻辑)
                // -----------------------------------------------------------
                // 法向：强制指向可行空间 (plane_n 方向)，忽略符号
                double rebound_speed = std::abs(post_n_scalar); 
                Eigen::Vector3d v_n_post = rebound_speed * plane_n;

                // 切向：沿原来的切向方向
                Eigen::Vector3d v_t_post = post_t_scalar * t_dir;

                // 这一步不需要做任何旋转还原，因为我们是在世界坐标系下用 plane_n 和 t_dir 重建的
                Eigen::Vector3d v_post = v_n_post + v_t_post;
                // v_post 的 Z 分量包含在 t_dir 或 v_n_post 中 (取决于墙面倾角)，不需要额外处理 input[2]

                // 保留原本的 z 轴覆盖逻辑? 
                // 注意：原代码的输出里并没有显式使用 output[2]，而是通过 v_n + v_t 合成
                // 如果 output[0] 和 [1] 已经包含了全部分量，则逻辑正确。

                if (!v_post.allFinite())
                {
                    ROS_ERROR("calculateDynamicPostCollisionVelocity: TRT produced non-finite output");
                    return Eigen::Vector3d::Zero();
                }
                return v_post;
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("calculateDynamicPostCollisionVelocity: TRT collisionBs1 exception: %s", e.what());
                return Eigen::Vector3d::Zero();
            }
        }
    };

    // 多段轨迹段信息
    struct TrajectorySegment
    {
        Eigen::Matrix<double, 3, 4> start_PVAJ;  // 起点状态(位置、速度、加速度、jerk)
        Eigen::Matrix<double, 3, 4> end_PVAJ;    // 终点状态
        std::vector<Eigen::Matrix<double, 6, -1>> corridor; // 该段的安全通道
        bool is_collision_segment;               // 是否为碰撞段
        CollisionEvent collision_event;          // 如果是碰撞段，存储碰撞信息

        // Optional initial guess from upstream (front-end):
        // - initial_path_waypoints: 3xM polyline (M>=2). It will be resampled to per-piece points.
        // - initial_segment_times: per-segment durations along that polyline (size M-1). If size != pieceN,
        //   we will use its total duration and distribute to pieces proportionally to geometric lengths.
        Eigen::Matrix3Xd initial_path_waypoints;
        Eigen::VectorXd initial_segment_times;
        
        TrajectorySegment() : is_collision_segment(false) {
            start_PVAJ.setZero();
            end_PVAJ.setZero();
        }
    };

    class COLLISION_GCOPTER
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::Matrix<double, 6, -1> PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
#if TRAJ_ORDER == 3
        minco::MINCO_S2NU collision_minco;
#elif TRAJ_ORDER == 5
        minco::MINCO_S3NU collision_minco;
#elif TRAJ_ORDER == 7
        minco::MINCO_S4NU collision_minco;
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

        // Optional upstream initial guess polyline used for initializing points.
        Eigen::Matrix3Xd initial_guess_path_;

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

        // 碰撞模型参数
        double friction_;                       // 摩擦系数（直观参数）
        double damping_ratio_;                  // 阻尼比（直观参数）
        
        // Calculations should use `friction_` and `damping_ratio_` directly.
        double max_collision_velocity_;         // 最大碰撞速度

        // 碰撞约束相关成员变量
        std::vector<CollisionEvent> collision_events_;     // 存储碰撞事件
        int collision_constraint_dim_;                     // 碰撞约束维度
        double collision_position_weight_;                 // 碰撞前位置约束权重
        double collision_velocity_weight_;                 // 碰撞前速度约束权重
        
        // 碰撞约束辅助变量（只针对碰撞前状态）
        Eigen::VectorXd collision_position_params_;        // 碰撞前位置参数u (2D for each collision)
        Eigen::VectorXd collision_position_grad_;          // 位置参数梯度

        // 末端约束放松相关变量
        bool enable_relaxed_end_constraint_;               // 是否启用末端约束放松
        Eigen::Matrix<double, 3, 4> original_tailPVAJ_;   // 原始的固定末端状态
        Eigen::VectorXd relaxed_end_position_params_;     // 松弛末端位置参数 (2D)
        Eigen::VectorXd relaxed_end_position_grad_;       // 松弛末端位置梯度
        double relaxed_end_position_weight_;              // 松弛末端位置约束权重
        
        // 新增：动态碰撞速度更新相关变量
        std::vector<Eigen::Vector3d> cached_post_collision_velocities_;  // 缓存的碰撞后速度
        bool enable_dynamic_collision_update_;            // 是否启用动态碰撞速度更新

        // ========== 联合优化（单碰撞双段）相关成员变量 ==========
        // pre 段 MINCO 和 post 段 MINCO 分别用 collision_minco（继承用）和 minco_post_
#if TRAJ_ORDER == 3
        minco::MINCO_S2NU minco_post_;
#elif TRAJ_ORDER == 5
        minco::MINCO_S3NU minco_post_;
#elif TRAJ_ORDER == 7
        minco::MINCO_S4NU minco_post_;
#endif

        // pre 段变量
        PolyhedraV vPolytopes_pre_;
        PolyhedraH hPolytopes_pre_;
        Eigen::Matrix3Xd shortPath_pre_;
        Eigen::VectorXi pieceIdx_pre_;
        Eigen::VectorXi vPolyIdx_pre_;
        Eigen::VectorXi hPolyIdx_pre_;
        int polyN_pre_, pieceN_pre_, spatialDim_pre_, temporalDim_pre_;
        Eigen::Matrix3Xd points_pre_;
        Eigen::VectorXd times_pre_;
        Eigen::Matrix3Xd gradByPoints_pre_;
        Eigen::VectorXd gradByTimes_pre_;
        Eigen::MatrixX3d partialGradByCoeffs_pre_;
        Eigen::VectorXd partialGradByTimes_pre_;
        Eigen::VectorXi resVector_pre_;

        // post 段变量
        PolyhedraV vPolytopes_post_;
        PolyhedraH hPolytopes_post_;
        Eigen::Matrix3Xd shortPath_post_;
        Eigen::VectorXi pieceIdx_post_;
        Eigen::VectorXi vPolyIdx_post_;
        Eigen::VectorXi hPolyIdx_post_;
        int polyN_post_, pieceN_post_, spatialDim_post_, temporalDim_post_;
        Eigen::Matrix3Xd points_post_;
        Eigen::VectorXd times_post_;
        Eigen::Matrix3Xd gradByPoints_post_;
        Eigen::VectorXd gradByTimes_post_;
        Eigen::MatrixX3d partialGradByCoeffs_post_;
        Eigen::VectorXd partialGradByTimes_post_;
        Eigen::VectorXi resVector_post_;

        // 联合优化边界条件
        Eigen::Matrix<double, 3, 4> headPVAJ_pre_;   // pre 段起点
        Eigen::Matrix<double, 3, 4> tailPVAJ_pre_;   // pre 段终点（碰撞点）
        Eigen::Matrix<double, 3, 4> headPVAJ_post_;  // post 段起点（碰撞点，速度由 TRT 给定）
        Eigen::Matrix<double, 3, 4> tailPVAJ_post_;  // post 段终点

        // TRT 软约束权重
        double trt_velocity_weight_;                 // TRT 碰撞后速度软约束权重


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
                                      &COLLISION_GCOPTER::costTinyNLS,
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
            
            // 特殊情况：单段轨迹（没有中间点）
            if (points.cols() == 0) {
                // 只有一段，直接计算起点到终点的距离
                double dis = (end - start).norm();
                resVector(0) = (int)(dis * integralRes + 1);

                return;
            }
            
            // 正常情况：有中间点的多段轨迹
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

        // 首先修改 costFunctional 函数，添加详细的调试信息
        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &grad)
        {
            COLLISION_GCOPTER &obj = *(COLLISION_GCOPTER *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            
            // === 步骤1：输入验证 ===
            if (!x.allFinite()) {
                ROS_ERROR("DEBUG: costFunctional input x contains NaN/Inf");
                return INFINITY;
            }
            
            // 检查维度
            int collision_count = obj.collision_events_.empty() ? 0 : 1;
            int collision_param_dim = collision_count * 3;
            int relaxed_end_param_dim = obj.enable_relaxed_end_constraint_ ? 2 : 0;
            int expected_size = dimTau + dimXi + collision_param_dim + relaxed_end_param_dim;
            
            if (x.size() != expected_size) {
                ROS_ERROR("DEBUG: Dimension mismatch! Expected %d, got %d", expected_size, (int)x.size());
                ROS_ERROR("  dimTau=%d, dimXi=%d, collision_param=%d, relaxed_end=%d", 
                        dimTau, dimXi, collision_param_dim, relaxed_end_param_dim);
                return INFINITY;
            }
            
            // === 步骤2：参数映射 ===
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(grad.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(grad.data() + dimTau, dimXi);
            
            // 碰撞约束参数映射（single-collision -> fixed small sizes）
            Eigen::Map<const Eigen::VectorXd> collision_pos_params(x.data() + dimTau + dimXi, collision_count * 2);
            Eigen::Map<const Eigen::VectorXd> collision_vel_params(x.data() + dimTau + dimXi + collision_count * 2, collision_count);
            Eigen::Map<Eigen::VectorXd> grad_collision_pos(grad.data() + dimTau + dimXi, collision_count * 2);
            Eigen::Map<Eigen::VectorXd> grad_collision_vel(grad.data() + dimTau + dimXi + collision_count * 2, collision_count);
            
            // 末端约束放松参数映射
            Eigen::Map<const Eigen::VectorXd> relaxed_end_pos_params(x.data() + dimTau + dimXi + collision_param_dim, obj.enable_relaxed_end_constraint_ ? 2 : 0);
            Eigen::Map<Eigen::VectorXd> grad_relaxed_end_pos(grad.data() + dimTau + dimXi + collision_param_dim, obj.enable_relaxed_end_constraint_ ? 2 : 0);

            // 检查映射后的参数
            if (!tau.allFinite()) {
                ROS_ERROR("DEBUG: tau contains NaN/Inf: min=%.6f, max=%.6f", tau.minCoeff(), tau.maxCoeff());
                return INFINITY;
            }
            
            if (!xi.allFinite()) {
                ROS_ERROR("DEBUG: xi contains NaN/Inf: min=%.6f, max=%.6f", xi.minCoeff(), xi.maxCoeff());
                return INFINITY;
            }
            
            if (!collision_pos_params.allFinite() || !collision_vel_params.allFinite()) {
                ROS_ERROR("DEBUG: collision params contain NaN/Inf");
                return INFINITY;
            }
            
            if (obj.enable_relaxed_end_constraint_) {
                if (!relaxed_end_pos_params.allFinite()) {
                    ROS_ERROR("DEBUG: relaxed end params contain NaN/Inf");
                    return INFINITY;
                }
            }

            // === 步骤3：前向变换 ===
            try {
                forwardT(tau, obj.times);
                
                if (!obj.times.allFinite()) {
                    ROS_ERROR("DEBUG: forwardT produced NaN/Inf times: min=%.6f, max=%.6f", obj.times.minCoeff(), obj.times.maxCoeff());
                    return INFINITY;
                }
                
                if (obj.times.minCoeff() <= 0) {
                    ROS_ERROR("DEBUG: forwardT produced non-positive times: min=%.6f", obj.times.minCoeff());
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: forwardT threw exception: %s", e.what());
                return INFINITY;
            }

            try {
                forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);
                
                if (!obj.points.allFinite()) {
                    ROS_ERROR("DEBUG: forwardP produced NaN/Inf points");
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: forwardP threw exception: %s", e.what());
                return INFINITY;
            }

            // === 步骤4：MINCO设置和能量计算 ===
            double cost = 0;
            try {
                obj.collision_minco.setParameters(obj.points, obj.times);
                
                obj.collision_minco.getEnergy(cost);
                
                if (!std::isfinite(cost)) {
                    ROS_ERROR("DEBUG: MINCO getEnergy produced non-finite cost: %.6f", cost);
                    return INFINITY;
                }
                
                if (cost < 0) {
                    ROS_ERROR("DEBUG: MINCO getEnergy produced negative cost: %.6f", cost);
                    return INFINITY;
                }
                
                
                obj.minctrl_cost = cost;
                obj.collision_minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
                obj.collision_minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
                
                if (!obj.partialGradByCoeffs.allFinite()) {
                    ROS_ERROR("DEBUG: partialGradByCoeffs contains NaN/Inf");
                    return INFINITY;
                }
                
                if (!obj.partialGradByTimes.allFinite()) {
                    ROS_ERROR("DEBUG: partialGradByTimes contains NaN/Inf");
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: MINCO operations threw exception: %s", e.what());
                return INFINITY;
            }
            
            // === 步骤5：权重应用 ===
            cost *= obj.penaltyWt(6);
            obj.minctrl_cost = cost;
            obj.partialGradByCoeffs *= obj.penaltyWt(6);
            obj.partialGradByTimes *= obj.penaltyWt(6);
            
            if (!std::isfinite(cost)) {
                ROS_ERROR("DEBUG: Cost became non-finite after penalty weight application: %.6f * %.6f", 
                        obj.minctrl_cost / obj.penaltyWt(6), obj.penaltyWt(6));
                return INFINITY;
            }
            

            // === 步骤6：附加惩罚函数 ===
            double pre_penalty_cost = cost;
            try {
                attachPenaltyFunctional(obj.times, obj.collision_minco.getCoeffs(),
                                        obj.hPolyIdx, obj.hPolytopes,
                                        obj.smoothEps, obj.resVector, obj.integralRes,
                                        obj.magnitudeBd, obj.penaltyWt, false,
                                        false, obj.flatmap, cost,
                                        obj.partialGradByTimes, obj.partialGradByCoeffs,
                                        obj.pos_cost, obj.vel_cost, obj.acc_cost, obj.omg_cost);
                
                if (!std::isfinite(cost)) {
                    ROS_ERROR("DEBUG: attachPenaltyFunctional produced non-finite cost: %.6f (was %.6f)", cost, pre_penalty_cost);
                    return INFINITY;
                }
                
                if (!obj.partialGradByCoeffs.allFinite() || !obj.partialGradByTimes.allFinite()) {
                    ROS_ERROR("DEBUG: attachPenaltyFunctional produced non-finite gradients");
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: attachPenaltyFunctional threw exception: %s", e.what());
                return INFINITY;
            }

            // === 步骤7：碰撞约束 ===
            double pre_collision_cost = cost;
            if (!obj.collision_events_.empty()) {
                try {
                    Trajectory<TRAJ_ORDER> current_traj;
                    obj.collision_minco.getTrajectory(current_traj);
                    
                    if (current_traj.getPieceNum() == 0) {
                        ROS_ERROR("DEBUG: collision_minco.getTrajectory returned empty trajectory");
                        return INFINITY;
                    }
                    
                    int collision_count = obj.collision_events_.empty() ? 0 : 1;
                    Eigen::VectorXd collision_pos_grad, collision_vel_grad;
                    collision_pos_grad.resize(collision_count * 2);
                    collision_vel_grad.resize(collision_count);
                    
                    // 7.a 先评估位置松弛约束（只影响碰撞参数梯度）
                    double collision_cost = obj.evaluateCollisionConstraints(
                        collision_pos_params, collision_vel_params, current_traj,
                        collision_pos_grad, collision_vel_grad);
                    
                    if (!std::isfinite(collision_cost)) {
                        ROS_ERROR("DEBUG: evaluateCollisionConstraints returned non-finite cost: %.6f", collision_cost);
                        return INFINITY;
                    }
                    
                    cost += collision_cost;
                    
                    // 7.b 在碰撞点加入速度罚，并将其梯度耦合到 MINCO 系数（仅一个时间点）
                    const auto &event = obj.collision_events_[0];

                    double traj_duration = current_traj.getTotalDuration();
                    double t_collision = event.collision_time;
                    if (t_collision < 0.2) {
                        t_collision = traj_duration;
                    }
                    if (t_collision < 0.0 || t_collision > traj_duration) {
                        t_collision = traj_duration;
                    }

                    // 仅当轨迹持续时间正且 times 有效时才计算
                    if (traj_duration > 0.0 && obj.times.size() > 0) {
                        // 找到碰撞所在的分段及其局部时间 s
                        const int pieceNum = obj.times.size();
                        int piece_idx = pieceNum - 1;
                        double t_acc = 0.0;
                        for (int i = 0; i < pieceNum; ++i) {
                            double t_next = t_acc + obj.times(i);
                            if (t_collision <= t_next || i == pieceNum - 1) {
                                piece_idx = i;
                                break;
                            }
                            t_acc = t_next;
                        }

                        double s_local = t_collision - t_acc;  // 该段内的局部时间
                        if (s_local < 0.0) s_local = 0.0;
                        if (s_local > obj.times(piece_idx)) s_local = obj.times(piece_idx);

                        // 计算该段在 s_local 处的速度及其对系数的梯度
                        Eigen::MatrixX3d coeffs = obj.collision_minco.getCoeffs();
                        const int stride = TRAJ_ORDER + 1;
                        if (coeffs.rows() >= (piece_idx + 1) * stride) {
                            Eigen::Matrix<double, TRAJ_ORDER + 1, 3> c =
                                coeffs.block<TRAJ_ORDER + 1, 3>(piece_idx * stride, 0);

                            double s1 = s_local;
                            double s2 = s1 * s1;
                            double s3 = s2 * s1;
                            double s4 = s2 * s2;
                            double s5 = s4 * s1;
                            double s6 = s3 * s3;
                            double s7 = s6 * s1;

                            Eigen::Matrix<double, TRAJ_ORDER + 1, 1> beta1;
#if TRAJ_ORDER == 3
                            beta1(0) = 0.0; beta1(1) = 1.0; beta1(2) = 2.0 * s1; beta1(3) = 3.0 * s2;
#elif TRAJ_ORDER == 5
                            beta1(0) = 0.0; beta1(1) = 1.0; beta1(2) = 2.0 * s1; beta1(3) = 3.0 * s2;
                            beta1(4) = 4.0 * s3; beta1(5) = 5.0 * s4;
#elif TRAJ_ORDER == 7
                            beta1(0) = 0.0; beta1(1) = 1.0; beta1(2) = 2.0 * s1; beta1(3) = 3.0 * s2;
                            beta1(4) = 4.0 * s3; beta1(5) = 5.0 * s4; beta1(6) = 6.0 * s5; beta1(7) = 7.0 * s6;
#endif

                            Eigen::Vector3d vel = c.transpose() * beta1;

                            Eigen::Vector3d n = event.plane_normal.normalized();
                            if (n.allFinite() && n.norm() > 1e-6) {
                                double v_n = vel.dot(n);
                                Eigen::Vector3d v_t_vec = vel - v_n * n;
                                double v_t = v_t_vec.norm();

                                // 法向与切向超限量
                                double viola_n = std::fabs(v_n) - obj.max_collision_velocity_;
                                double viola_t = v_t - obj.max_collision_velocity_;

                                double pena_n = 0.0, pena_n_d = 0.0;
                                double pena_t = 0.0, pena_t_d = 0.0;

                                Eigen::Vector3d grad_v = Eigen::Vector3d::Zero();

                                if (std::isfinite(viola_n) &&
                                    smoothedL1(viola_n, obj.smoothEps, pena_n, pena_n_d)) {
                                    if (!std::isfinite(pena_n)) {
                                        return INFINITY;
                                    }
                                    cost += obj.collision_velocity_weight_ * pena_n;

                                    double sign_n = (v_n >= 0.0) ? 1.0 : -1.0;
                                    grad_v += obj.collision_velocity_weight_ * pena_n_d * sign_n * n;
                                }

                                if (std::isfinite(viola_t) &&
                                    smoothedL1(viola_t, obj.smoothEps, pena_t, pena_t_d)) {
                                    if (!std::isfinite(pena_t)) {
                                        return INFINITY;
                                    }
                                    cost += obj.collision_velocity_weight_ * pena_t;

                                    if (v_t > 1e-6) {
                                        Eigen::Vector3d t_dir = v_t_vec / v_t;
                                        grad_v += obj.collision_velocity_weight_ * pena_t_d * t_dir;
                                    }
                                }

                                // 将速度梯度映射到该段的系数梯度（仅一个采样点，无时间缩放因子）
                                if (grad_v.allFinite()) {
                                    obj.partialGradByCoeffs.block(piece_idx * stride, 0,
                                                                  stride, 3) += beta1 * grad_v.transpose();
                                }
                            }
                        }
                    }

                    if (!std::isfinite(cost)) {
                        ROS_ERROR("DEBUG: Cost became non-finite after adding collision cost: %.6f + %.6f", pre_collision_cost, collision_cost);
                        return INFINITY;
                    }

                    grad_collision_pos = collision_pos_grad;
                    grad_collision_vel = collision_vel_grad;
                    
                } catch (const std::exception& e) {
                    ROS_ERROR("DEBUG: Collision constraints threw exception: %s", e.what());
                    return INFINITY;
                }
            } else {
                grad_collision_pos.setZero();
                grad_collision_vel.setZero();
            }
            
            // === 步骤8：末端约束放松 ===
            double pre_relaxed_cost = cost;
            if (obj.enable_relaxed_end_constraint_) {
                try {
                    Trajectory<TRAJ_ORDER> current_traj;
                    obj.collision_minco.getTrajectory(current_traj);
                    
                    Eigen::VectorXd relaxed_end_pos_grad;
                    relaxed_end_pos_grad.resize(2);

                    double relaxed_end_cost = obj.evaluateRelaxedEndConstraints(
                        relaxed_end_pos_params, current_traj,
                        relaxed_end_pos_grad);
                    
                    if (!std::isfinite(relaxed_end_cost)) {
                        ROS_ERROR("DEBUG: evaluateRelaxedEndConstraints returned non-finite cost: %.6f", relaxed_end_cost);
                        return INFINITY;
                    }
                    
                    cost += relaxed_end_cost;
                    
                    if (!std::isfinite(cost)) {
                        ROS_ERROR("DEBUG: Cost became non-finite after adding relaxed end cost: %.6f + %.6f", pre_relaxed_cost, relaxed_end_cost);
                        return INFINITY;
                    }
                    
                    grad_relaxed_end_pos = relaxed_end_pos_grad;
                    
                } catch (const std::exception& e) {
                    ROS_ERROR("DEBUG: Relaxed end constraints threw exception: %s", e.what());
                    return INFINITY;
                }
            } else {
                if (relaxed_end_param_dim > 0) {
                    grad_relaxed_end_pos.setZero();
                }
            }

            // === 步骤9：梯度传播 ===
            try {
                obj.collision_minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                        obj.gradByPoints, obj.gradByTimes);
                
                if (!obj.gradByPoints.allFinite() || !obj.gradByTimes.allFinite()) {
                    ROS_ERROR("DEBUG: propogateGrad produced non-finite gradients");
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: propogateGrad threw exception: %s", e.what());
                return INFINITY;
            }

            // === 步骤10：时间代价 ===
            obj.time_cost = weightT * obj.times.sum();
            cost += obj.time_cost;
            obj.gradByTimes.array() += weightT;
            
            if (!std::isfinite(cost)) {
                ROS_ERROR("DEBUG: Cost became non-finite after adding time cost: %.6f", obj.time_cost);
                return INFINITY;
            }

            // === 步骤11：反向梯度传播 ===
            try {
                backwardGradT(tau, obj.gradByTimes, gradTau);
                if (!gradTau.allFinite()) {
                    ROS_ERROR("DEBUG: backwardGradT produced non-finite gradients");
                    return INFINITY;
                }
                
                backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
                if (!gradXi.allFinite()) {
                    ROS_ERROR("DEBUG: backwardGradP produced non-finite gradients");
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: Backward gradient propagation threw exception: %s", e.what());
                return INFINITY;
            }
            
            // === 步骤12：标准化限制层 ===
            try {
                normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);
                
                if (!std::isfinite(cost) || !gradXi.allFinite()) {
                    ROS_ERROR("DEBUG: normRetrictionLayer produced non-finite values, cost=%.6f", cost);
                    return INFINITY;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("DEBUG: normRetrictionLayer threw exception: %s", e.what());
                return INFINITY;
            }

            // === 最终检查 ===
            if (!grad.allFinite()) {
                ROS_ERROR("DEBUG: Final gradient contains NaN/Inf");
                return INFINITY;
            }
            
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
            COLLISION_GCOPTER &obj = *(COLLISION_GCOPTER *)ptr;
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

                obj.collision_minco.getTrajectory(obj.Traj);
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
                                  &COLLISION_GCOPTER::costDistance,
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

        // 碰撞约束相关函数
        inline void initializeCollisionConstraints(const std::vector<CollisionEvent>& events,
                                                   double position_weight = 10.0,
                                                   double velocity_weight = 50.0)
        {
            // 单碰撞模式：只保留第一个事件（若有），并初始化固定大小的碰撞参数
            collision_events_.clear();
            if (!events.empty()) {
                if (events.size() > 1) {
                    ROS_WARN("initializeCollisionConstraints: received %lu events, keeping only the first (single-collision mode).", events.size());
                }
                collision_events_.push_back(events.front());
            }

            collision_constraint_dim_ = collision_events_.empty() ? 0 : 3; // 2D position + 1D velocity

            if (collision_events_.empty()) {
                collision_position_params_.resize(0);
                collision_position_grad_.resize(0);
            } else {
                // 初始化第一个事件的参数与约束
                collision_events_[0].initializeConstraintParameters();

                collision_position_params_.resize(2);
                collision_position_grad_.resize(2);

                collision_position_params_.setZero();
            }

            // 设置约束权重（使用传入的配置参数）
            collision_position_weight_ = position_weight;
            collision_velocity_weight_ = velocity_weight;
        }
        
        // 计算碰撞前点位置（松弛约束） - 单事件版本（忽略索引，只使用第一个事件）
        inline Eigen::Vector3d calculatePreCollisionPointPosition(int /*collision_idx*/, const Eigen::Vector2d& u_param) const
        {
            if (collision_events_.empty()) {
                return Eigen::Vector3d::Zero();
            }

            const auto& event = collision_events_[0];

            // 确保||u|| ≤ 1的约束
            Eigen::Vector2d u_constrained = u_param;
            double u_norm = u_constrained.norm();
            if (u_norm > 1.0) {
                u_constrained = u_constrained / u_norm;
            }

            // 计算松弛位置：p_c = p_0 + R*B*r*u
            Eigen::Vector3d offset = event.rotation_matrix * event.basis_matrix * (event.max_position_offset * u_constrained);
            return event.reference_point + offset;
        }


        // 初始化末端约束放松
        inline void initializeRelaxedEndConstraints(bool enable = true,
                                                   double position_weight = 10.0)
        {
            enable_relaxed_end_constraint_ = enable;
            
            if (enable) {
                relaxed_end_position_params_.resize(2);
                relaxed_end_position_grad_.resize(2);

                // 初始化为0（保持在原始末端状态附近）
                relaxed_end_position_params_.setZero();

                relaxed_end_position_weight_ = position_weight;

                // 保存原始末端状态
                original_tailPVAJ_ = tailPVAJ;
            } else {
                relaxed_end_position_params_.resize(0);
                relaxed_end_position_grad_.resize(0);
            }
        }

        // 计算松弛的末端位置
        inline Eigen::Vector3d calculateRelaxedEndPosition(const Eigen::Vector2d& u_param) const
        {
            if (!enable_relaxed_end_constraint_ || collision_events_.empty()) {
                return original_tailPVAJ_.col(0);
            }
            
            // 使用第一个碰撞事件的约束参数（适用于末端为碰撞点的情况）
            const auto& event = collision_events_[0];
            
            // 确保||u|| ≤ 1的约束
            Eigen::Vector2d u_constrained = u_param;
            double u_norm = u_constrained.norm();
            if (u_norm > 1.0) {
                u_constrained = u_constrained / u_norm;
            }
            
            // 计算松弛末端位置：基于原始末端位置的偏移
            Eigen::Vector3d offset = event.rotation_matrix * event.basis_matrix * (event.max_position_offset * u_constrained);
            return original_tailPVAJ_.col(0) + offset;
        }



        // 评估末端约束放松的代价和梯度
        inline double evaluateRelaxedEndConstraints(
            const Eigen::VectorXd& end_pos_params,
            const Trajectory<TRAJ_ORDER>& current_traj,
            Eigen::VectorXd& pos_grad) const
        {
            double total_cost = 0.0;
            pos_grad.setZero();

            if (!enable_relaxed_end_constraint_) {
                return total_cost;
            }

            // 获取当前参数（仅位置参数）
            Eigen::Vector2d u_param = end_pos_params.segment<2>(0);

            // 计算预测的末端位置（末端速度不再松弛）
            Eigen::Vector3d predicted_pos = calculateRelaxedEndPosition(u_param);

            // 获取轨迹实际的末端状态
            double traj_duration = current_traj.getTotalDuration();
            Eigen::Vector3d actual_pos = current_traj.getPos(traj_duration);

            // 末端位置约束代价（仅位置松弛）
            Eigen::Vector3d pos_error = actual_pos - predicted_pos;
            double pos_cost = relaxed_end_position_weight_ * pos_error.squaredNorm();
            total_cost += pos_cost;

            // 位置梯度计算
            if (!collision_events_.empty()) {
                const auto& event = collision_events_[0];
                Eigen::Vector2d pos_grad_local = -2.0 * relaxed_end_position_weight_ * 
                    (event.basis_matrix.transpose() * event.rotation_matrix.transpose() * pos_error * event.max_position_offset);

                // 应用||u|| ≤ 1约束的梯度修正
                double u_norm = u_param.norm();
                if (u_norm > 1.0 && u_norm > 1e-6) {
                    Eigen::Matrix2d projection = Eigen::Matrix2d::Identity() - u_param * u_param.transpose() / (u_norm * u_norm);
                    pos_grad_local = projection * pos_grad_local / u_norm;
                }

                pos_grad.segment<2>(0) = pos_grad_local;
            }

            return total_cost;
        }

        // 评估碰撞前约束代价和梯度
        inline double evaluateCollisionConstraints(
            const Eigen::VectorXd& collision_pos_params,
            const Eigen::VectorXd& collision_vel_params,
            const Trajectory<TRAJ_ORDER>& current_traj,
            Eigen::VectorXd& pos_grad,
            Eigen::VectorXd& vel_grad) const
        {
            double total_cost = 0.0;
            pos_grad.setZero();
            vel_grad.setZero();
            
            if (collision_events_.empty()) {
                return total_cost;
            }

            // 单事件处理（只考虑第一个碰撞事件）
            const auto& event = collision_events_[0];

            // 获取当前参数（single-collision -> index 0）
            Eigen::Vector2d u_param = collision_pos_params.segment<2>(0);

            // 基本检查
            if (!u_param.allFinite()) {
                return INFINITY;
            }

            // 计算预测的碰撞前点位置和速度
            Eigen::Vector3d predicted_pos = calculatePreCollisionPointPosition(0, u_param);
            if (!predicted_pos.allFinite()) {
                return INFINITY;
            }

            // 动态调整碰撞时间到段末端
            double t_collision = event.collision_time;
            double traj_duration = current_traj.getTotalDuration();

            if (t_collision < 0.2) {
                t_collision = traj_duration;
            }
            if (t_collision < 0 || t_collision > traj_duration) {
                t_collision = traj_duration;
            }

            Eigen::Vector3d actual_pos = current_traj.getPos(t_collision);
            Eigen::Vector3d actual_vel = current_traj.getVel(t_collision);

            if (!actual_pos.allFinite() || !actual_vel.allFinite()) {
                return INFINITY;
            }

            // 法向速度的有符号分量
            double actual_normal_vel_signed = actual_vel.dot(event.plane_normal);
            if (!std::isfinite(actual_normal_vel_signed)) {
                return INFINITY;
            }

            // 碰撞前位置约束代价
            Eigen::Vector3d pos_error = actual_pos - predicted_pos;
            double pos_cost = collision_position_weight_ * pos_error.squaredNorm();
            if (!std::isfinite(pos_cost)) {
                return INFINITY;
            }
            total_cost += pos_cost;

            // 位置梯度计算
            Eigen::Vector2d pos_grad_local = -2.0 * collision_position_weight_ *
                (event.basis_matrix.transpose() * event.rotation_matrix.transpose() * pos_error * event.max_position_offset);

            if (!pos_grad_local.allFinite()) {
                return INFINITY;
            }

            double u_norm = u_param.norm();
            if (u_norm > 1.0) {
                if (u_norm > 1e-6) {
                    Eigen::Matrix2d projection = Eigen::Matrix2d::Identity() - u_param * u_param.transpose() / (u_norm * u_norm);
                    pos_grad_local = projection * pos_grad_local / u_norm;
                    if (!pos_grad_local.allFinite()) {
                        return INFINITY;
                    }
                } else {
                    pos_grad_local.setZero();
                }
            }

            pos_grad.segment<2>(0) += pos_grad_local;
            
            if (!std::isfinite(total_cost) || !pos_grad.allFinite() || !vel_grad.allFinite()) {
                return INFINITY;
            }
            
            return total_cost;
        }

        static inline bool isPointInHPoly(const PolyhedronH &hPoly,
                                          const Eigen::Vector3d &pt,
                                          const double eps = 1.0e-6)
        {
            for (int i = 0; i < hPoly.cols(); ++i)
            {
                const Eigen::Vector3d n = hPoly.col(i).head<3>();
                const Eigen::Vector3d p0 = hPoly.col(i).tail<3>();
                const double val = n.dot(pt - p0);
                if (val > eps)
                {
                    return false;
                }
            }
            return true;
        }

        inline PolyhedronH getHPolyForVPolyIndex(const int vpoly_idx) const
        {
            if (vpoly_idx < 0)
            {
                return PolyhedronH(6, 0);
            }
            if ((vpoly_idx % 2) == 0)
            {
                const int i = vpoly_idx / 2;
                if (i < 0 || i >= static_cast<int>(hPolytopes.size()))
                {
                    return PolyhedronH(6, 0);
                }
                return hPolytopes[i];
            }
            else
            {
                const int i = (vpoly_idx - 1) / 2;
                if (i < 0 || (i + 1) >= static_cast<int>(hPolytopes.size()))
                {
                    return PolyhedronH(6, 0);
                }
                const auto &a = hPolytopes[i];
                const auto &b = hPolytopes[i + 1];
                PolyhedronH inter(6, a.cols() + b.cols());
                inter.leftCols(a.cols()) = a;
                inter.rightCols(b.cols()) = b;
                return inter;
            }
        }

        static inline Eigen::Matrix3Xd resamplePolylineByArcLength(const Eigen::Matrix3Xd &path,
                                                                    const int sample_count)
        {
            Eigen::Matrix3Xd out;
            if (sample_count <= 0)
            {
                out.resize(3, 0);
                return out;
            }
            if (path.cols() < 2)
            {
                out.resize(3, 0);
                return out;
            }

            Eigen::VectorXd s(path.cols());
            s(0) = 0.0;
            for (int i = 1; i < path.cols(); ++i)
            {
                s(i) = s(i - 1) + (path.col(i) - path.col(i - 1)).norm();
            }
            const double total = s(path.cols() - 1);
            out.resize(3, sample_count);

            if (!std::isfinite(total) || total <= 1.0e-9)
            {
                // Degenerate path: fall back to linear interpolation.
                for (int k = 0; k < sample_count; ++k)
                {
                    const double a = (sample_count == 1) ? 0.0 : (double)k / (double)(sample_count - 1);
                    out.col(k) = (1.0 - a) * path.col(0) + a * path.col(path.cols() - 1);
                }
                return out;
            }

            int seg = 0;
            for (int k = 0; k < sample_count; ++k)
            {
                const double target = (sample_count == 1) ? 0.0 : total * (double)k / (double)(sample_count - 1);
                while (seg + 1 < s.size() && s(seg + 1) < target)
                {
                    seg++;
                }
                if (seg + 1 >= s.size())
                {
                    out.col(k) = path.col(path.cols() - 1);
                    continue;
                }
                const double seg_len = s(seg + 1) - s(seg);
                const double u = (seg_len <= 1.0e-12) ? 0.0 : (target - s(seg)) / seg_len;
                out.col(k) = (1.0 - u) * path.col(seg) + u * path.col(seg + 1);
            }
            return out;
        }

        inline bool applyInitialGuessPathToPoints(Eigen::Matrix3Xd &inout_points) const
        {
            if (initial_guess_path_.cols() < 2)
            {
                return false;
            }
            if (pieceN < 2)
            {
                return false;
            }
            // Sample pieceN+1 points, then use interior ones as per-piece inner points.
            Eigen::Matrix3Xd path = initial_guess_path_;
            // Enforce endpoints consistent with boundary conditions.
            path.col(0) = headPVAJ.col(0);
            path.col(path.cols() - 1) = tailPVAJ.col(0);

            Eigen::Matrix3Xd sampled = resamplePolylineByArcLength(path, pieceN + 1);
            if (sampled.cols() != pieceN + 1)
            {
                return false;
            }
            Eigen::Matrix3Xd pts = sampled.middleCols(1, pieceN - 1);
            if (pts.cols() != pieceN - 1)
            {
                return false;
            }

            // Ensure each point lies in its expected corridor polytope/intersection.
            for (int j = 0; j < pts.cols(); ++j)
            {
                const int vp = vPolyIdx(j);
                PolyhedronH h = getHPolyForVPolyIndex(vp);
                if (h.cols() <= 0)
                {
                    continue;
                }
                if (!isPointInHPoly(h, pts.col(j), 1.0e-6))
                {
                    Eigen::Vector3d interior;
                    if (geoutils::findInterior(h, interior))
                    {
                        pts.col(j) = interior;
                    }
                }
            }

            inout_points = pts;
            return true;
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
                          const Eigen::VectorXd &initial_segment_times = Eigen::VectorXd(),
                          const Eigen::Matrix3Xd &initial_guess_path = Eigen::Matrix3Xd(),
                          const double &friction = 0.65,
                          const double &damping_ratio = 0.10,
                          const double &max_collision_vel = 3.0,
                          const std::vector<CollisionEvent> &collision_events = std::vector<CollisionEvent>(),
                          const double &position_constraint_weight = 10.0,
                          const double &velocity_constraint_weight = 50.0,
                          const bool &enable_relaxed_end = false,
                          const double &relaxed_end_position_weight = 10.0)
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

            // store optional upstream initial guess polyline for initializing points
            initial_guess_path_ = initial_guess_path;

            // 初始化碰撞模型参数：以更直观的物理参数 friction / damping_ratio 为主
            friction_ = friction;
            damping_ratio_ = damping_ratio;
            max_collision_velocity_ = max_collision_vel;  // 最大碰撞速度 (m/s)

            // 初始化碰撞约束（使用配置参数）
            if (!collision_events.empty()) {
                initializeCollisionConstraints(collision_events, 
                                              position_constraint_weight,
                                              velocity_constraint_weight);
            } else {
                // 清空之前段的碰撞事件
                collision_events_.clear();
                collision_constraint_dim_ = 0;
                collision_position_params_.resize(0);
                collision_position_grad_.resize(0);
            }

            // 初始化末端约束放松（如果启用且为碰撞段）
            if (enable_relaxed_end && !collision_events.empty()) {
                initializeRelaxedEndConstraints(true, relaxed_end_position_weight);
            } else {
                initializeRelaxedEndConstraints(false);
            }

            // 新增：初始化动态碰撞速度更新
            enable_dynamic_collision_update_ = !collision_events.empty();
            cached_post_collision_velocities_.clear();

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

            // 设置轨迹边界条件（末端可能松弛）
            Eigen::Matrix<double, 3, 4> effective_tailPVAJ = tailPVAJ;
            
            // 如果启用末端约束放松，则在每次优化迭代中动态更新末端状态
            // 这里先使用原始末端状态进行初始化，实际约束在cost函数中处理

#if TRAJ_ORDER == 3
            collision_minco.setConditions(headPVAJ.leftCols(2), effective_tailPVAJ.leftCols(2), pieceN);
#elif TRAJ_ORDER == 5
            collision_minco.setConditions(headPVAJ.leftCols(3), effective_tailPVAJ.leftCols(3), pieceN);
#elif TRAJ_ORDER == 7
            collision_minco.setConditions(headPVAJ, effective_tailPVAJ, pieceN);
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
            ROS_INFO("Starting optimize function with relCostTol=%.6f", relCostTol);
            ROS_INFO("  pieceN=%d, temporalDim=%d, spatialDim=%d", pieceN, temporalDim, spatialDim);
            ROS_INFO("  collision_events_.size()=%lu", collision_events_.size());
            
            Traj = traj;
            
            // 计算包含碰撞约束和末端约束放松的优化变量总维度（single-collision mode）
            int collision_count = collision_events_.empty() ? 0 : 1;
            int collision_param_dim = collision_count * 3; // 2D position + 1D velocity per collision
            int relaxed_end_param_dim = enable_relaxed_end_constraint_ ? 2 : 0; // 2D position only for end
            Eigen::VectorXd x(temporalDim + spatialDim + collision_param_dim + relaxed_end_param_dim);
            
            // 映射传统优化变量
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);
            
            // 映射碰撞约束参数（single-collision）
            Eigen::Map<Eigen::VectorXd> collision_pos_params(x.data() + temporalDim + spatialDim, collision_count * 2);
            Eigen::Map<Eigen::VectorXd> collision_vel_params(x.data() + temporalDim + spatialDim + collision_count * 2, collision_count);
            
            // 映射末端约束放松参数
            Eigen::Map<Eigen::VectorXd> relaxed_end_pos_params(x.data() + temporalDim + spatialDim + collision_param_dim, enable_relaxed_end_constraint_ ? 2 : 0);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);

            // If upstream initial guess polyline provided, override initial inner points.
            if (initial_guess_path_.cols() >= 2)
            {
                if (applyInitialGuessPathToPoints(points))
                {
                    ROS_INFO("COLLISION_GCOPTER: using provided initial guess path to initialize points");
                }
                else
                {
                    ROS_WARN("COLLISION_GCOPTER: initial guess path provided but failed to apply; using corridor shortest-path init");
                }
            }

            // If initial per-segment times provided by upstream (sample), map to piece times
            if (initial_segment_times_.size() > 0) {
                if (initial_segment_times_.size() == pieceN) {
                    times = initial_segment_times_;
                    ROS_INFO("COLLISION_GCOPTER: using provided per-piece initial times (size==pieceN)");
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
                        ROS_INFO("COLLISION_GCOPTER: distributed provided total time %.3f to pieces (len_sum=%.3f)", provided_total, len_sum);
                    } else {
                        ROS_WARN("COLLISION_GCOPTER: provided initial_segment_times sum is zero, keeping default times");
                    }
                }
            }
            
            // 检查初始化数据
            if (!points.allFinite()) {
                ROS_ERROR("Initial points contain NaN or Inf");
                return INFINITY;
            }
            
            if (!times.allFinite()) {
                ROS_ERROR("Initial times contain NaN or Inf");
                return INFINITY;
            }
            
            for (int i = 0; i < times.size(); ++i) {
                if (times(i) <= 0) {
                    ROS_ERROR("Invalid initial time[%d]: %.6f", i, times(i));
                    return INFINITY;
                }
            }
            
            ROS_INFO("Initial setup completed successfully:");
            ROS_INFO("  points: %dx%d, times: %d", (int)points.rows(), (int)points.cols(), (int)times.size());
            ROS_INFO("  total initial duration: %.3f seconds", times.sum());
            
            // set sample point number
            setResVector(headPVAJ.col(0), tailPVAJ.col(0), points, times, integralRes, resVector);
            
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);
            
            // 检查转换后的优化变量
            if (!tau.allFinite()) {
                ROS_ERROR("Initial tau contains NaN or Inf");
                return INFINITY;
            }
            
            if (!xi.allFinite()) {
                ROS_ERROR("Initial xi contains NaN or Inf");
                return INFINITY;
            }
            
            // 初始化碰撞约束参数
            if (!collision_events_.empty()) {
                // 检查存储的碰撞约束参数（仅位置参数，速度参数已弃用）
                if (collision_position_params_.size() != (collision_count * 2) ||
                    !collision_position_params_.allFinite()) {
                    return INFINITY;
                }

                collision_pos_params = collision_position_params_;
                // velocity param (zeta) deprecated: initialize to zero
                collision_vel_params.setZero();
            }
            
            // 初始化末端约束放松参数
            if (enable_relaxed_end_constraint_) {
                // 检查存储的末端约束参数（仅位置参数）
                if (relaxed_end_position_params_.size() != static_cast<size_t>(2) ||
                    !relaxed_end_position_params_.allFinite()) {
                    return INFINITY;
                }

                relaxed_end_pos_params = relaxed_end_position_params_;
            }
            
            // 初始代价函数评估
            Eigen::VectorXd initial_grad(x.size());
            ROS_INFO("Evaluating initial cost function with x.size()=%d", (int)x.size());
            double initial_cost = costFunctional(this, x, initial_grad);
            
            if (!std::isfinite(initial_cost) || !initial_grad.allFinite()) {
                ROS_ERROR("Initial cost evaluation failed: cost=%.6f, grad_finite=%s", 
                         initial_cost, initial_grad.allFinite() ? "true" : "false");
                return INFINITY;
            }
            
            ROS_INFO("Initial cost evaluation successful: cost=%.6f", initial_cost);

            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;

            // 测试一次代价函数调用
            Eigen::VectorXd test_grad(x.size());
            ROS_INFO("DEBUG: Testing cost function before L-BFGS...");
            double test_cost = costFunctional(this, x, test_grad);
            ROS_INFO("DEBUG: Test cost function result: cost=%.6f, grad_finite=%s, grad_norm=%.6f", 
                    test_cost, test_grad.allFinite() ? "true" : "false", test_grad.norm());

            if (!std::isfinite(test_cost) || !test_grad.allFinite()) {
                ROS_ERROR("DEBUG: Cost function test failed before L-BFGS!");
                return INFINITY;
}
            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &COLLISION_GCOPTER::costFunctional,
                                            nullptr,
                                            &COLLISION_GCOPTER::costProgress,
                                            this,
                                            lbfgs_params);

            ROS_INFO("L-BFGS optimization completed with return code: %d, final cost: %.6f", ret, minCostFunctional);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                
                // 检查最终的参数
                if (!times.allFinite() || !points.allFinite()) {
                    return INFINITY;
                }
                
                collision_minco.setParameters(points, times);
                collision_minco.getTrajectory(traj);
                
                // 新增：动态更新碰撞后速度
                if (enable_dynamic_collision_update_ && !collision_events_.empty()) {
                    updatePostCollisionVelocities(traj);
                }
                
                // 更新碰撞约束参数
                if (!collision_events_.empty()) {
                    collision_position_params_ = collision_pos_params;
                }
                
                // 更新末端约束放松参数
                if (enable_relaxed_end_constraint_) {
                    relaxed_end_position_params_ = relaxed_end_pos_params;
                }
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
            }

            return minCostFunctional;
        }

        // 多段碰撞轨迹优化接口
        inline bool optimizeMultiSegmentTrajectory(
            const std::vector<TrajectorySegment>& segments,
            const double& relCostTol,
            const double& timeWeight,
            const double& lengthPerPiece,
            const double& smoothingFactor,
            const int& integralResolution,
            const int& flipResolution,
            const Eigen::VectorXd& magnitudeBounds,
            const Eigen::VectorXd& penaltyWeights,
            const bool& isDebug,
            Visualizer& visual,
            ros::Publisher& optPuber,
            std::vector<Trajectory<TRAJ_ORDER>>& segment_trajectories,
            const double& position_constraint_weight = 1.0,
            const double& velocity_constraint_weight = 5.0,
            const double& friction = 0.65,
            const double& damping_ratio = 0.10,
            const double& max_collision_velocity = 3.0,
            const double& trt_velocity_weight = 3e5)  // 新增：TRT 软约束权重
        {
            if (segments.empty()) {
                return false;
            }

            segment_trajectories.clear();

            // ===== 检测单碰撞情况：恰好两段，第一段是碰撞段，第二段是普通段 =====
            if (segments.size() == 2 && segments[0].is_collision_segment && !segments[1].is_collision_segment) {
                ROS_INFO("optimizeMultiSegmentTrajectory: Detected single-collision case, using joint optimization");
                
                Trajectory<TRAJ_ORDER> traj_pre, traj_post;
                bool success = optimizeCollisionTrajectoryJoint(
                    segments[0],  // pre (collision segment)
                    segments[1],  // post (normal segment)
                    relCostTol,
                    timeWeight,
                    lengthPerPiece,
                    smoothingFactor,
                    integralResolution,
                    magnitudeBounds,
                    penaltyWeights,
                    isDebug,
                    visual,
                    optPuber,
                    friction,
                    damping_ratio,
                    max_collision_velocity,
                    trt_velocity_weight,
                    position_constraint_weight,
                    traj_pre,
                    traj_post);

                if (!success) {
                    ROS_ERROR("optimizeMultiSegmentTrajectory: Joint optimization failed");
                    return false;
                }

                segment_trajectories.push_back(traj_pre);
                segment_trajectories.push_back(traj_post);
                return true;
            }

            // ===== 其他情况：使用原来的串行优化逻辑 =====
            ROS_INFO("optimizeMultiSegmentTrajectory: Using sequential optimization for %zu segments", segments.size());;
            segment_trajectories.reserve(segments.size());

            // 设置约束权重
            collision_position_weight_ = position_constraint_weight;
            collision_velocity_weight_ = velocity_constraint_weight;

            // 验证所有段的基本有效性
            for (size_t i = 0; i < segments.size(); ++i) {
                const auto& segment = segments[i];
                
                // 检查基本数据有效性
                if (!segment.start_PVAJ.allFinite() || !segment.end_PVAJ.allFinite() || 
                    segment.corridor.empty()) {
                    return false;
                }
                
                // 检查碰撞事件数据（如果是碰撞段）
                if (segment.is_collision_segment) {
                    const auto& collision = segment.collision_event;
                    
                    if (!collision.collision_point.allFinite() || 
                        !collision.plane_normal.allFinite() || 
                        !collision.pre_collision_velocity.allFinite() || 
                        !collision.post_collision_velocity.allFinite()) {
                        return false;
                    }
                    
                    // collision_time为-1.0是正常的（表示段末端）
                    if (collision.collision_time != -1.0 && 
                        (collision.collision_time < 0 || !std::isfinite(collision.collision_time))) {
                        return false;
                    }
                }
            }

            // 为每一段分别优化轨迹，实时更新后续段的起始状态
            for (size_t i = 0; i < segments.size(); ++i) {
                auto segment = segments[i];  // 复制一份，因为可能需要修改

                // === 关键修改：根据前一段的实际优化结果更新当前段的起始状态 ===
                if (i > 0) {
                    // 获取前一段轨迹的末端状态
                    const auto& prev_traj = segment_trajectories[i - 1];
                    double prev_duration = prev_traj.getTotalDuration();
                    
                    // 更新当前段的起始位置为前一段的末端位置
                    Eigen::Vector3d actual_end_position = prev_traj.getPos(prev_duration);
                    segment.start_PVAJ.col(0) = actual_end_position;
                    
                    // 更新当前段的起始速度
                    Eigen::Vector3d actual_end_velocity;
                    if (segments[i - 1].is_collision_segment) {
                        // 原来：calculateDynamicPostCollisionVelocity
                        // actual_end_velocity = segments[i - 1].collision_event.calculateDynamicPostCollisionVelocity(prev_traj, -1.0, friction, damping_ratio);

                        // 改成：
                        Eigen::Vector3d v_pre = prev_traj.getVel(prev_duration);
                        const auto &evt = segments[i - 1].collision_event;

                        {
                            // Use dynamic black-box predictor to compute post-collision velocity
                            actual_end_velocity = evt.calculateDynamicPostCollisionVelocity(prev_traj, -1.0, friction, damping_ratio, evt.given_yaw);
                        }
                        ROS_INFO("Segment %zu: bounce pre=(%.3f,%.3f,%.3f) → post=(%.3f,%.3f,%.3f)",
                                i,
                                v_pre.x(), v_pre.y(), v_pre.z(),
                                actual_end_velocity.x(), actual_end_velocity.y(), actual_end_velocity.z());
                    }else {
                        // 如果前一段是普通段，直接使用末端速度
                        actual_end_velocity = prev_traj.getVel(prev_duration);
                        
                        ROS_INFO("Segment %lu: Updated start position from normal segment %lu", i + 1, i);
                        ROS_INFO("  Previous segment end position: (%.3f, %.3f, %.3f)", 
                                 actual_end_position.x(), actual_end_position.y(), actual_end_position.z());
                        ROS_INFO("  Previous segment end velocity: (%.3f, %.3f, %.3f)",
                                 actual_end_velocity.x(), actual_end_velocity.y(), actual_end_velocity.z());
                    }
                    
                    segment.start_PVAJ.col(1) = actual_end_velocity;
                    
                    // 清零高阶导数（加速度、jerk）以避免不连续性
                    segment.start_PVAJ.col(2).setZero();  // 加速度
                    segment.start_PVAJ.col(3).setZero();  // jerk
                    
                    // 验证更新后的起始状态
                    if (!segment.start_PVAJ.col(0).allFinite() || !segment.start_PVAJ.col(1).allFinite()) {
                        ROS_ERROR("Segment %lu: Invalid updated start state", i + 1);
                        return false;
                    }
                }

                // 设置该段的边界条件
                Eigen::Matrix<double, 3, 4> segmentStartPVAJ = segment.start_PVAJ;
                Eigen::Matrix<double, 3, 4> segmentEndPVAJ = segment.end_PVAJ;

                std::vector<Eigen::Matrix<double, 6, -1>> segment_corridor = segment.corridor;

                // 简化的占位符参数
                Eigen::VectorXd dummyUseKeyPos = Eigen::VectorXd::Zero(1);
                Eigen::Matrix3Xd dummyKeyAtt = Eigen::Matrix3Xd::Zero(3, 1);
                Eigen::Matrix3Xd dummyKeyPos = Eigen::Matrix3Xd::Zero(3, 1);
                Eigen::VectorXd dummyKeyTime = Eigen::VectorXd::Zero(1);

                // 准备碰撞事件（仅针对碰撞段）
                std::vector<CollisionEvent> segment_collision_events;
                if (segment.is_collision_segment) {
                    // 处理"碰撞在段末端"的情况
                    CollisionEvent adjusted_event = segment.collision_event;
                    if (adjusted_event.collision_time == -1.0) {
                        // 碰撞在段末端，设置为一个占位符时间（将在优化过程中动态调整）
                        adjusted_event.collision_time = 0.1;  // 占位符，实际会动态调整到轨迹末端
                    }
                    
                    segment_collision_events.push_back(adjusted_event);
                }

                // 设置该段的优化器，对碰撞段启用末端约束放松
                bool enable_relaxed_end = segment.is_collision_segment;
                if (!setup(timeWeight, segmentStartPVAJ, segmentEndPVAJ,
                          segment_corridor, lengthPerPiece, smoothingFactor, 
                          integralResolution, flipResolution,
                          magnitudeBounds, penaltyWeights, dummyUseKeyPos,
                          dummyKeyAtt, dummyKeyPos, dummyKeyTime, isDebug,
                          false, visual, optPuber,
                          /* initial_segment_times = */ segment.initial_segment_times,
                          /* initial_guess_path = */ segment.initial_path_waypoints,
                          friction, damping_ratio,
                          max_collision_velocity,
                          segment_collision_events,  // 只有碰撞段才有碰撞事件
                          position_constraint_weight, velocity_constraint_weight,
                          enable_relaxed_end, position_constraint_weight))
                {
                    ROS_ERROR("Multi-segment optimization: setup failed for segment %lu", i + 1);
                    return false;
                }

                // 优化该段轨迹
                Trajectory<TRAJ_ORDER> segment_traj;
                int optFailedCount = 0;
                double optimization_cost = INFINITY;
                
                while (optFailedCount < 5) {
                    optimization_cost = optimize(segment_traj, relCostTol);
                    
                    if (optimization_cost < INFINITY) {
                        break; // 优化成功
                    }
                    
                    optFailedCount++;
                }

                if (optimization_cost >= INFINITY) {
                    ROS_ERROR("Multi-segment optimization: failed to optimize segment %lu after %d attempts", i + 1, optFailedCount);
                    return false;
                }

                // 验证优化后的轨迹
                if (segment_traj.getPieceNum() == 0 || segment_traj.getTotalDuration() <= 0) {
                    ROS_ERROR("Multi-segment optimization: segment %lu produced invalid trajectory (pieces=%d, duration=%.3f)", 
                              i + 1, segment_traj.getPieceNum(), segment_traj.getTotalDuration());
                    return false;
                }

                // 存储优化后的轨迹段
                segment_trajectories.push_back(segment_traj);
                
                // 显示当前段的优化结果
                double segment_duration = segment_traj.getTotalDuration();
                Eigen::Vector3d segment_start_pos = segment_traj.getPos(0.0);
                Eigen::Vector3d segment_start_vel = segment_traj.getVel(0.0);
                Eigen::Vector3d segment_end_pos = segment_traj.getPos(segment_duration);
                Eigen::Vector3d segment_end_vel = segment_traj.getVel(segment_duration);
                
                ROS_INFO("Segment %lu optimization completed:", i + 1);
                ROS_INFO("  Start: pos=(%.3f,%.3f,%.3f), vel=(%.3f,%.3f,%.3f)", 
                         segment_start_pos.x(), segment_start_pos.y(), segment_start_pos.z(),
                         segment_start_vel.x(), segment_start_vel.y(), segment_start_vel.z());
                ROS_INFO("  End:   pos=(%.3f,%.3f,%.3f), vel=(%.3f,%.3f,%.3f)", 
                         segment_end_pos.x(), segment_end_pos.y(), segment_end_pos.z(),
                         segment_end_vel.x(), segment_end_vel.y(), segment_end_vel.z());
                ROS_INFO("  Duration: %.3f seconds", segment_duration);
                
                                // 如果是碰撞段，显示碰撞后速度信息
                                if (segment.is_collision_segment) {
                                    Eigen::Vector3d dynamic_post_collision_velocity = Eigen::Vector3d::Zero();
                                    double friction_local = friction_;
                                    double damping_local = damping_ratio_;
                                    dynamic_post_collision_velocity =
                                        segment.collision_event.calculateDynamicPostCollisionVelocity(segment_traj, -1.0, friction_local, damping_local, segment.collision_event.given_yaw);
                                    ROS_INFO("  Post-collision velocity: (%.3f, %.3f, %.3f)",
                                             dynamic_post_collision_velocity.x(), 
                                             dynamic_post_collision_velocity.y(), 
                                             dynamic_post_collision_velocity.z());
                                }
            }

            // === 新增：验证轨迹段之间的连续性 ===
            ROS_INFO("=== Trajectory Segment Continuity Verification ===");
            bool continuity_valid = true;
            
            for (size_t idx = 0; idx < segment_trajectories.size(); ++idx) {
                const auto &seg = segment_trajectories[idx];
                // 起始速度
                Eigen::Vector3d v_start = seg.getVel(0.0);
                Eigen::Vector3d p_start = seg.getPos(0.0);
                // 末端速度
                double t_end = seg.getTotalDuration();
                Eigen::Vector3d v_end   = seg.getVel(t_end);
                Eigen::Vector3d p_end   = seg.getPos(t_end);
                
                ROS_INFO("Segment %zu: start_pos [%.3f, %.3f, %.3f], start_vel [%.3f, %.3f, %.3f]",
                         idx+1,
                         p_start.x(), p_start.y(), p_start.z(),
                         v_start.x(), v_start.y(), v_start.z());
                ROS_INFO("          end_pos   [%.3f, %.3f, %.3f], end_vel   [%.3f, %.3f, %.3f]",
                         p_end.x(), p_end.y(), p_end.z(),
                         v_end.x(), v_end.y(), v_end.z());
                
                // 检查与下一段的连续性
                if (idx < segment_trajectories.size() - 1) {
                    const auto &next_seg = segment_trajectories[idx + 1];
                    Eigen::Vector3d next_p_start = next_seg.getPos(0.0);
                    Eigen::Vector3d next_v_start = next_seg.getVel(0.0);
                    
                    // 位置连续性检查
                    double pos_discontinuity = (p_end - next_p_start).norm();
                    if (pos_discontinuity > 1e-2) {  // 1cm 容差
                        ROS_WARN("Position discontinuity between segment %zu and %zu: %.6f m", 
                                 idx+1, idx+2, pos_discontinuity);
                        continuity_valid = false;
                    }
                    
                    // 速度连续性检查（考虑碰撞段的速度跳跃）
                    if (segments[idx].is_collision_segment) {
                        // 碰撞段：检查碰撞后速度与下一段起始速度的一致性
                       // 也改成直接对上一段末端速度做反弹
                       // 计算期望的碰撞后速度（确保变量在后续作用域可用）
                       Eigen::Vector3d expected_post_collision_vel =
                           segments[idx].collision_event.calculateDynamicPostCollisionVelocity(seg, -1.0, friction, damping_ratio, segments[idx].collision_event.given_yaw);

                        double vel_discontinuity = (expected_post_collision_vel - next_v_start).norm();
                        if (vel_discontinuity > 0.1) {  // 0.1 m/s 容差
                            ROS_WARN("Post-collision velocity discontinuity between segment %zu and %zu: %.6f m/s", 
                                     idx+1, idx+2, vel_discontinuity);
                            ROS_INFO("  Expected post-collision: [%.3f, %.3f, %.3f]", 
                                     expected_post_collision_vel.x(), expected_post_collision_vel.y(), expected_post_collision_vel.z());
                            ROS_INFO("  Actual next start vel:   [%.3f, %.3f, %.3f]", 
                                     next_v_start.x(), next_v_start.y(), next_v_start.z());
                            continuity_valid = false;
                        } else {
                            ROS_INFO("Post-collision velocity continuity OK (error: %.6f m/s)", vel_discontinuity);
                        }
                    } else {
                        // 普通段：检查末端速度与下一段起始速度的一致性
                        double vel_discontinuity = (v_end - next_v_start).norm();
                        if (vel_discontinuity > 0.1) {  // 0.1 m/s 容差
                            ROS_WARN("Velocity discontinuity between segment %zu and %zu: %.6f m/s", 
                                     idx+1, idx+2, vel_discontinuity);
                            continuity_valid = false;
                        } else {
                            ROS_INFO("  ✓ Velocity continuity OK (error: %.6f m/s)", vel_discontinuity);
                        }
                    }
                }
            }
            
            if (continuity_valid) {
                ROS_INFO("=== All trajectory segments are continuous ===");
            } else {
                ROS_WARN("=== Some trajectory segments have continuity issues ===");
            }

            return true;
        }

        // 拼接多段轨迹为单一轨迹
        inline bool concatenateTrajectories(
            const std::vector<Trajectory<TRAJ_ORDER>>& segment_trajectories,
            Trajectory<TRAJ_ORDER>& final_trajectory)
        {
            if (segment_trajectories.empty()) {
                return false;
            }

            // 收集所有段的系数矩阵和持续时间
            std::vector<typename Piece<TRAJ_ORDER>::CoefficientMat> all_coeffs;
            std::vector<double> all_durations;

            for (const auto& traj : segment_trajectories) {
                for (int i = 0; i < traj.getPieceNum(); ++i) {
                    all_coeffs.push_back(traj[i].getCoeffMat());
                    all_durations.push_back(traj[i].getDuration());
                }
            }

            // 创建拼接后的轨迹
            final_trajectory = Trajectory<TRAJ_ORDER>(all_durations, all_coeffs);

            return true;
        }
        
        // 新增：动态更新碰撞后速度
        inline void updatePostCollisionVelocities(const Trajectory<TRAJ_ORDER>& current_traj)
        {
            cached_post_collision_velocities_.clear();
            if (collision_events_.empty()) {
                return;
            }

            cached_post_collision_velocities_.reserve(1);
            const auto& event = collision_events_[0];
            {
                double friction_local = friction_;
                double damping_local = damping_ratio_;
                Eigen::Vector3d post_vel = event.calculateDynamicPostCollisionVelocity(current_traj, event.collision_time, friction_local, damping_local, event.given_yaw);
                cached_post_collision_velocities_.push_back(post_vel);
            }         
        }
        
        // 新增：获取动态更新的碰撞后速度
        inline Eigen::Vector3d getDynamicPostCollisionVelocity(size_t event_index) const
        {
            if (event_index >= cached_post_collision_velocities_.size()) {
                return Eigen::Vector3d::Zero();
            }
            return cached_post_collision_velocities_[event_index];
        }
        
        // 新增：启用/禁用动态碰撞速度更新
        inline void enableDynamicCollisionUpdate(bool enable = true)
        {
            enable_dynamic_collision_update_ = enable;
        }

        // Optional upstream initial guess for joint (pre+post) optimization
        Eigen::Matrix3Xd initial_guess_path_pre_;
        Eigen::Matrix3Xd initial_guess_path_post_;
        Eigen::VectorXd initial_segment_times_pre_;
        Eigen::VectorXd initial_segment_times_post_;

        // ========== 联合优化（单碰撞双段）核心接口 ==========

        /**
         * @brief 设置联合优化问题（单碰撞：pre 段 + post 段）
         * @param pre_corridor   pre 段走廊
         * @param post_corridor  post 段走廊
         * @param start_PVAJ     全局起点状态
         * @param collision_pt   碰撞点位置
         * @param end_PVAJ       全局终点状态
         * @param collision_event 碰撞事件（含平面法向、前端 pre/post 速度等）
         * @param ... 其他优化参数
         * @return 是否成功
         */
        inline bool setupJointOptimization(
            const PolyhedraH& pre_corridor,
            const PolyhedraH& post_corridor,
            const Eigen::Matrix<double, 3, 4>& start_PVAJ,
            const Eigen::Vector3d& collision_pt,
            const Eigen::Matrix<double, 3, 4>& end_PVAJ,
            const CollisionEvent& collision_event,
            const Eigen::VectorXd& pre_initial_segment_times,
            const Eigen::Matrix3Xd& pre_initial_guess_path,
            const Eigen::VectorXd& post_initial_segment_times,
            const Eigen::Matrix3Xd& post_initial_guess_path,
            const double& timeWeight,
            const double& lengthPerPiece,
            const double& smoothingFactor,
            const int& integralResolution,
            const Eigen::VectorXd& magnitudeBounds,
            const Eigen::VectorXd& penaltyWeights,
            const double& friction,
            const double& damping_ratio,
            const double& max_collision_vel,
            const double& trt_velocity_weight,
            const double& position_constraint_weight,
            const bool& isDebug,
            Visualizer& visual,
            ros::Publisher& optPuber)
        {
            isdebug = isDebug;
            visualTraj = &visual;
            optPub = &optPuber;
            rho = timeWeight;
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            allocSpeed = magnitudeBd(0);
            friction_ = friction;
            damping_ratio_ = damping_ratio;
            max_collision_velocity_ = max_collision_vel;
            trt_velocity_weight_ = trt_velocity_weight;
            collision_position_weight_ = position_constraint_weight;

            initial_segment_times_pre_ = pre_initial_segment_times;
            initial_guess_path_pre_ = pre_initial_guess_path;
            initial_segment_times_post_ = post_initial_segment_times;
            initial_guess_path_post_ = post_initial_guess_path;

            // 存储碰撞事件（单碰撞）
            collision_events_.clear();
            collision_events_.push_back(collision_event);
            collision_events_[0].initializeConstraintParameters();

            // ===== 设置 pre 段 =====
            headPVAJ_pre_ = start_PVAJ;
            tailPVAJ_pre_.setZero();
            tailPVAJ_pre_.col(0) = collision_pt;
            // 方案B：使用前端给定的 pre_collision_velocity 作为初始猜测
            // 注意：这里不再作为硬边界锁定，而是在迭代过程中允许优化变化
            tailPVAJ_pre_.col(1) = collision_event.pre_collision_velocity;
            tailPVAJ_pre_.col(3).setZero(); // ensure jerk = 0 at pre-end

            hPolytopes_pre_ = pre_corridor;
            for (size_t i = 0; i < hPolytopes_pre_.size(); i++) {
                hPolytopes_pre_[i].topRows<3>().colwise().normalize();
            }
            if (!processCorridor(hPolytopes_pre_, vPolytopes_pre_)) {
                ROS_ERROR("setupJointOptimization: pre corridor process failed");
                return false;
            }
            polyN_pre_ = hPolytopes_pre_.size();
            getShortestPath(headPVAJ_pre_.col(0), tailPVAJ_pre_.col(0), vPolytopes_pre_, smoothEps, shortPath_pre_);
            
            Eigen::Matrix3Xd deltas_pre = shortPath_pre_.rightCols(polyN_pre_) - shortPath_pre_.leftCols(polyN_pre_);
            pieceIdx_pre_ = (deltas_pre.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            pieceIdx_pre_.array() += 1;
            pieceN_pre_ = pieceIdx_pre_.sum();
            temporalDim_pre_ = pieceN_pre_;
            spatialDim_pre_ = 0;
            vPolyIdx_pre_.resize(pieceN_pre_ - 1);
            hPolyIdx_pre_.resize(pieceN_pre_);
            resVector_pre_.resize(pieceN_pre_);
            for (int i = 0, j = 0, k; i < polyN_pre_; i++) {
                k = pieceIdx_pre_(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        vPolyIdx_pre_(j) = 2 * i;
                        spatialDim_pre_ += vPolytopes_pre_[2 * i].cols();
                    } else if (i < polyN_pre_ - 1) {
                        vPolyIdx_pre_(j) = 2 * i + 1;
                        spatialDim_pre_ += vPolytopes_pre_[2 * i + 1].cols();
                    }
                    hPolyIdx_pre_(j) = i;
                }
            }
#if TRAJ_ORDER == 3
            collision_minco.setConditions(headPVAJ_pre_.leftCols(2), tailPVAJ_pre_.leftCols(2), pieceN_pre_);
#elif TRAJ_ORDER == 5
            collision_minco.setConditions(headPVAJ_pre_.leftCols(3), tailPVAJ_pre_.leftCols(3), pieceN_pre_);
#elif TRAJ_ORDER == 7
            collision_minco.setConditions(headPVAJ_pre_, tailPVAJ_pre_, pieceN_pre_);
#endif
            points_pre_.resize(3, pieceN_pre_ - 1);
            times_pre_.resize(pieceN_pre_);
            gradByPoints_pre_.resize(3, pieceN_pre_ - 1);
            gradByTimes_pre_.resize(pieceN_pre_);
            partialGradByCoeffs_pre_.resize((TRAJ_ORDER + 1) * pieceN_pre_, 3);
            partialGradByTimes_pre_.resize(pieceN_pre_);

            // ===== 设置 post 段 =====
            headPVAJ_post_.setZero();
            headPVAJ_post_.col(0) = collision_pt;
            // 方案B：使用前端给定的 post_collision_velocity 作为初始猜测
            // 在迭代过程中会根据TRT计算结果动态更新
            headPVAJ_post_.col(1) = collision_event.post_collision_velocity; 
            headPVAJ_post_.col(3).setZero(); // ensure jerk = 0 at post-start
            tailPVAJ_post_ = end_PVAJ;

            hPolytopes_post_ = post_corridor;
            for (size_t i = 0; i < hPolytopes_post_.size(); i++) {
                hPolytopes_post_[i].topRows<3>().colwise().normalize();
            }
            if (!processCorridor(hPolytopes_post_, vPolytopes_post_)) {
                ROS_ERROR("setupJointOptimization: post corridor process failed");
                return false;
            }
            polyN_post_ = hPolytopes_post_.size();
            getShortestPath(headPVAJ_post_.col(0), tailPVAJ_post_.col(0), vPolytopes_post_, smoothEps, shortPath_post_);
            
            Eigen::Matrix3Xd deltas_post = shortPath_post_.rightCols(polyN_post_) - shortPath_post_.leftCols(polyN_post_);
            pieceIdx_post_ = (deltas_post.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            pieceIdx_post_.array() += 1;
            pieceN_post_ = pieceIdx_post_.sum();
            temporalDim_post_ = pieceN_post_;
            spatialDim_post_ = 0;
            vPolyIdx_post_.resize(pieceN_post_ - 1);
            hPolyIdx_post_.resize(pieceN_post_);
            resVector_post_.resize(pieceN_post_);
            for (int i = 0, j = 0, k; i < polyN_post_; i++) {
                k = pieceIdx_post_(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        vPolyIdx_post_(j) = 2 * i;
                        spatialDim_post_ += vPolytopes_post_[2 * i].cols();
                    } else if (i < polyN_post_ - 1) {
                        vPolyIdx_post_(j) = 2 * i + 1;
                        spatialDim_post_ += vPolytopes_post_[2 * i + 1].cols();
                    }
                    hPolyIdx_post_(j) = i;
                }
            }
#if TRAJ_ORDER == 3
            minco_post_.setConditions(headPVAJ_post_.leftCols(2), tailPVAJ_post_.leftCols(2), pieceN_post_);
#elif TRAJ_ORDER == 5
            minco_post_.setConditions(headPVAJ_post_.leftCols(3), tailPVAJ_post_.leftCols(3), pieceN_post_);
#elif TRAJ_ORDER == 7
            minco_post_.setConditions(headPVAJ_post_, tailPVAJ_post_, pieceN_post_);
#endif
            points_post_.resize(3, pieceN_post_ - 1);
            times_post_.resize(pieceN_post_);
            gradByPoints_post_.resize(3, pieceN_post_ - 1);
            gradByTimes_post_.resize(pieceN_post_);
            partialGradByCoeffs_post_.resize((TRAJ_ORDER + 1) * pieceN_post_, 3);
            partialGradByTimes_post_.resize(pieceN_post_);

            // 碰撞位置松弛参数
            collision_position_params_.resize(2);
            collision_position_params_.setZero();
            collision_position_grad_.resize(2);

            ROS_INFO("setupJointOptimization: pre_pieceN=%d, post_pieceN=%d, trt_weight=%.2f",
                     pieceN_pre_, pieceN_post_, trt_velocity_weight_);
            return true;
        }

        /**
         * @brief 联合优化的代价函数（静态，供 L-BFGS 调用）
         * 优化变量布局：[tau_pre | xi_pre | tau_post | xi_post | v_pre_end(3) | collision_pos_u(1)]
         *
         * 说明：
         * - 过去 pre 段末端速度是 MINCO 的硬边界（setConditions），不在优化变量里，因此优化后必然与前端给定值一致。
         * - 这里将 pre 段末端速度 v_pre_end 加入优化变量，并在每次 cost 评估时重设 setConditions。
         * - v_pre_end 的梯度通过 MINCO 边界解析梯度回传（避免有限差分）。
         */
        static inline double costFunctionalJoint(void* ptr,
                                                  const Eigen::VectorXd& x,
                                                  Eigen::VectorXd& grad)
        {
            COLLISION_GCOPTER& obj = *(COLLISION_GCOPTER*)ptr;

            const int dimTau_pre = obj.temporalDim_pre_;
            const int dimXi_pre = obj.spatialDim_pre_;
            const int dimTau_post = obj.temporalDim_post_;
            const int dimXi_post = obj.spatialDim_post_;
            const int dimPreVel = 3; // pre 段末端速度
            const int dimCollision = 1; // 碰撞位置松弛 u（左右线段）

            const int expected_size = dimTau_pre + dimXi_pre + dimTau_post + dimXi_post + dimPreVel + dimCollision;
            if (x.size() != expected_size) {
                ROS_ERROR("costFunctionalJoint: dimension mismatch, expected %d, got %d", expected_size, (int)x.size());
                return INFINITY;
            }
            if (!x.allFinite()) {
                return INFINITY;
            }

            grad.setZero();

            // 映射优化变量
            int offset = 0;
            Eigen::Map<const Eigen::VectorXd> tau_pre(x.data() + offset, dimTau_pre); offset += dimTau_pre;
            Eigen::Map<const Eigen::VectorXd> xi_pre(x.data() + offset, dimXi_pre); offset += dimXi_pre;
            Eigen::Map<const Eigen::VectorXd> tau_post(x.data() + offset, dimTau_post); offset += dimTau_post;
            Eigen::Map<const Eigen::VectorXd> xi_post(x.data() + offset, dimXi_post); offset += dimXi_post;
            Eigen::Map<const Eigen::VectorXd> v_pre_end(x.data() + offset, dimPreVel); offset += dimPreVel;
            Eigen::Map<const Eigen::VectorXd> collision_u(x.data() + offset, dimCollision);

            offset = 0;
            Eigen::Map<Eigen::VectorXd> grad_tau_pre(grad.data() + offset, dimTau_pre); offset += dimTau_pre;
            Eigen::Map<Eigen::VectorXd> grad_xi_pre(grad.data() + offset, dimXi_pre); offset += dimXi_pre;
            Eigen::Map<Eigen::VectorXd> grad_tau_post(grad.data() + offset, dimTau_post); offset += dimTau_post;
            Eigen::Map<Eigen::VectorXd> grad_xi_post(grad.data() + offset, dimXi_post); offset += dimXi_post;
            Eigen::Map<Eigen::VectorXd> grad_v_pre_end(grad.data() + offset, dimPreVel); offset += dimPreVel;
            Eigen::Map<Eigen::VectorXd> grad_collision_u(grad.data() + offset, dimCollision);

            double cost = 0.0;

            const Eigen::Vector3d v_pre_end_vec = v_pre_end.head<3>();
            if (!v_pre_end_vec.allFinite()) {
                return INFINITY;
            }

            if (obj.collision_events_.empty()) {
                return INFINITY;
            }

            const auto &event = obj.collision_events_[0];
            const double u_param = collision_u[0];
            if (!std::isfinite(u_param)) return INFINITY;
            if (!event.reference_point.allFinite() ||
                !event.rotation_matrix.allFinite() ||
                !event.basis_matrix.allFinite() ||
                !std::isfinite(event.max_position_offset)) {
                return INFINITY;
            }

            // collision point is an optimization variable along ONE tangential direction (left-right line segment)
            // Choose the more "horizontal" tangential basis (smaller |dot(z)|) to avoid up/down motion in vertical planes.
            const Eigen::Vector3d world_z(0.0, 0.0, 1.0);
            const Eigen::Vector3d t0 = event.rotation_matrix * event.basis_matrix.col(0);
            const Eigen::Vector3d t1 = event.rotation_matrix * event.basis_matrix.col(1);
            const int t_idx = (std::abs(t0.dot(world_z)) <= std::abs(t1.dot(world_z))) ? 0 : 1;
            const Eigen::Vector3d M_dir = (event.rotation_matrix * event.basis_matrix.col(t_idx)) * event.max_position_offset;

            double u_constrained = u_param;
            double ducon_du = 1.0;
            if (u_param > 1.0) {
                u_constrained = 1.0;
                ducon_du = 0.0;
            } else if (u_param < -1.0) {
                u_constrained = -1.0;
                ducon_du = 0.0;
            }

            const Eigen::Vector3d collision_pt_eval = event.reference_point + M_dir * u_constrained;
            if (!collision_pt_eval.allFinite()) {
                return INFINITY;
            }

            // 约束：碰撞前速度幅值不超过 max_collision_velocity_
            // 方法同全局最大速度限制：使用 smoothL1 惩罚
            // 权重复用 trt_velocity_weight_（TRT 软约束已移除，避免增加新参数）
            {
                const double vnorm = v_pre_end_vec.norm();
                if (std::isfinite(vnorm) && vnorm > 1e-9) {
                    const double viola = vnorm - obj.max_collision_velocity_;
                    if (viola > 0.0 && std::isfinite(obj.trt_velocity_weight_) && obj.trt_velocity_weight_ > 0.0) {
                        double f = 0.0, df = 0.0;
                        smoothedL1(viola, obj.smoothEps, f, df);
                        if (!std::isfinite(f) || !std::isfinite(df)) {
                            return INFINITY;
                        }
                        cost += obj.trt_velocity_weight_ * f;
                    }
                }
            }

            // ===== Pre 段 =====
            // v_pre_end 是优化变量，因此这里每次评估都需要重设 MINCO 边界条件
            Eigen::Matrix<double, 3, 4> tailPVAJ_pre_eval = obj.tailPVAJ_pre_;
            tailPVAJ_pre_eval.col(0) = collision_pt_eval;
            tailPVAJ_pre_eval.col(1) = v_pre_end_vec;
            tailPVAJ_pre_eval.col(3).setZero(); // ensure jerk = 0 for eval copy
#if TRAJ_ORDER == 3
            obj.collision_minco.setConditions(obj.headPVAJ_pre_.leftCols(2), tailPVAJ_pre_eval.leftCols(2), obj.pieceN_pre_);
#elif TRAJ_ORDER == 5
            obj.collision_minco.setConditions(obj.headPVAJ_pre_.leftCols(3), tailPVAJ_pre_eval.leftCols(3), obj.pieceN_pre_);
#elif TRAJ_ORDER == 7
            obj.collision_minco.setConditions(obj.headPVAJ_pre_, tailPVAJ_pre_eval, obj.pieceN_pre_);
#endif

            forwardT(tau_pre, obj.times_pre_);
            if (!obj.times_pre_.allFinite() || obj.times_pre_.minCoeff() <= 0) return INFINITY;
            forwardP(xi_pre, obj.vPolyIdx_pre_, obj.vPolytopes_pre_, obj.points_pre_);
            if (!obj.points_pre_.allFinite()) return INFINITY;

            obj.collision_minco.setParameters(obj.points_pre_, obj.times_pre_);
            double energy_pre = 0.0;
            obj.collision_minco.getEnergy(energy_pre);
            if (!std::isfinite(energy_pre)) return INFINITY;
            cost += obj.penaltyWt(6) * energy_pre;

            obj.collision_minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs_pre_);
            obj.collision_minco.getEnergyPartialGradByTimes(obj.partialGradByTimes_pre_);
            obj.partialGradByCoeffs_pre_ *= obj.penaltyWt(6);
            obj.partialGradByTimes_pre_ *= obj.penaltyWt(6);

            // attachPenaltyFunctional for pre
            double pos_cost_pre = 0, vel_cost_pre = 0, acc_cost_pre = 0, omg_cost_pre = 0;
            attachPenaltyFunctional(obj.times_pre_, obj.collision_minco.getCoeffs(),
                                    obj.hPolyIdx_pre_, obj.hPolytopes_pre_,
                                    obj.smoothEps, obj.resVector_pre_, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, false, false, obj.flatmap,
                                    cost, obj.partialGradByTimes_pre_, obj.partialGradByCoeffs_pre_,
                                    pos_cost_pre, vel_cost_pre, acc_cost_pre, omg_cost_pre);

            // 时间代价 pre
            cost += obj.rho * obj.times_pre_.sum();
            obj.partialGradByTimes_pre_.array() += obj.rho;

            // 同时获取 MINCO 对边界条件（head/tail PVA(J)）的解析梯度，用于 v_pre_end
#if TRAJ_ORDER == 3
            Eigen::Matrix<double, 3, 2> gradByHeadPV_pre;
            Eigen::Matrix<double, 3, 2> gradByTailPV_pre;
            obj.collision_minco.propogateGrad(obj.partialGradByCoeffs_pre_, obj.partialGradByTimes_pre_,
                                               obj.gradByPoints_pre_, obj.gradByTimes_pre_,
                                               gradByHeadPV_pre, gradByTailPV_pre);
#elif TRAJ_ORDER == 5
            Eigen::Matrix3d gradByHeadPVA_pre;
            Eigen::Matrix3d gradByTailPVA_pre;
            obj.collision_minco.propogateGrad(obj.partialGradByCoeffs_pre_, obj.partialGradByTimes_pre_,
                                               obj.gradByPoints_pre_, obj.gradByTimes_pre_,
                                               gradByHeadPVA_pre, gradByTailPVA_pre);
#elif TRAJ_ORDER == 7
            Eigen::Matrix<double, 3, 4> gradByHeadPVAJ_pre;
            Eigen::Matrix<double, 3, 4> gradByTailPVAJ_pre;
            obj.collision_minco.propogateGrad(obj.partialGradByCoeffs_pre_, obj.partialGradByTimes_pre_,
                                               obj.gradByPoints_pre_, obj.gradByTimes_pre_,
                                               gradByHeadPVAJ_pre, gradByTailPVAJ_pre);
#endif
            backwardGradT(tau_pre, obj.gradByTimes_pre_, grad_tau_pre);
            backwardGradP(xi_pre, obj.vPolyIdx_pre_, obj.vPolytopes_pre_, obj.gradByPoints_pre_, grad_xi_pre);
            normRetrictionLayer(xi_pre, obj.vPolyIdx_pre_, obj.vPolytopes_pre_, cost, grad_xi_pre);

            // v_pre_end 的解析梯度来自 MINCO 对 tailPVAJ.col(1) 的梯度
            Eigen::Vector3d grad_collision_pos_from_pre = Eigen::Vector3d::Zero();
#if TRAJ_ORDER == 3
            grad_v_pre_end = gradByTailPV_pre.col(1);
            grad_collision_pos_from_pre = gradByTailPV_pre.col(0);
#elif TRAJ_ORDER == 5
            grad_v_pre_end = gradByTailPVA_pre.col(1);
            grad_collision_pos_from_pre = gradByTailPVA_pre.col(0);
#elif TRAJ_ORDER == 7
            grad_v_pre_end = gradByTailPVAJ_pre.col(1);
            grad_collision_pos_from_pre = gradByTailPVAJ_pre.col(0);
#endif

            // 加上显式的速度幅值上限 smoothL1 惩罚的梯度
            {
                const double vnorm = v_pre_end_vec.norm();
                if (std::isfinite(vnorm) && vnorm > 1e-9) {
                    const double viola = vnorm - obj.max_collision_velocity_;
                    if (viola > 0.0 && std::isfinite(obj.trt_velocity_weight_) && obj.trt_velocity_weight_ > 0.0) {
                        double f = 0.0, df = 0.0;
                        smoothedL1(viola, obj.smoothEps, f, df);
                        if (!std::isfinite(df)) {
                            return INFINITY;
                        }
                        grad_v_pre_end += obj.trt_velocity_weight_ * df * (v_pre_end_vec / vnorm);
                    }
                }
            }

            // ===== Post 段 =====
            // collision point is the start position of post segment, also an optimization variable
            Eigen::Matrix<double, 3, 4> headPVAJ_post_eval = obj.headPVAJ_post_;
            headPVAJ_post_eval.col(0) = collision_pt_eval;
#if TRAJ_ORDER == 3
            obj.minco_post_.setConditions(headPVAJ_post_eval.leftCols(2), obj.tailPVAJ_post_.leftCols(2), obj.pieceN_post_);
#elif TRAJ_ORDER == 5
            obj.minco_post_.setConditions(headPVAJ_post_eval.leftCols(3), obj.tailPVAJ_post_.leftCols(3), obj.pieceN_post_);
#elif TRAJ_ORDER == 7
            obj.minco_post_.setConditions(headPVAJ_post_eval, obj.tailPVAJ_post_, obj.pieceN_post_);
#endif

            forwardT(tau_post, obj.times_post_);
            if (!obj.times_post_.allFinite() || obj.times_post_.minCoeff() <= 0) return INFINITY;
            forwardP(xi_post, obj.vPolyIdx_post_, obj.vPolytopes_post_, obj.points_post_);
            if (!obj.points_post_.allFinite()) return INFINITY;

            obj.minco_post_.setParameters(obj.points_post_, obj.times_post_);
            double energy_post = 0.0;
            obj.minco_post_.getEnergy(energy_post);
            if (!std::isfinite(energy_post)) return INFINITY;
            cost += obj.penaltyWt(6) * energy_post;

            obj.minco_post_.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs_post_);
            obj.minco_post_.getEnergyPartialGradByTimes(obj.partialGradByTimes_post_);
            obj.partialGradByCoeffs_post_ *= obj.penaltyWt(6);
            obj.partialGradByTimes_post_ *= obj.penaltyWt(6);

            // attachPenaltyFunctional for post
            double pos_cost_post = 0, vel_cost_post = 0, acc_cost_post = 0, omg_cost_post = 0;
            attachPenaltyFunctional(obj.times_post_, obj.minco_post_.getCoeffs(),
                                    obj.hPolyIdx_post_, obj.hPolytopes_post_,
                                    obj.smoothEps, obj.resVector_post_, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, false, false, obj.flatmap,
                                    cost, obj.partialGradByTimes_post_, obj.partialGradByCoeffs_post_,
                                    pos_cost_post, vel_cost_post, acc_cost_post, omg_cost_post);

            // 时间代价 post
            cost += obj.rho * obj.times_post_.sum();
            obj.partialGradByTimes_post_.array() += obj.rho;

            // Also get boundary gradients for post head position (collision point)
            Eigen::Vector3d grad_collision_pos_from_post = Eigen::Vector3d::Zero();
#if TRAJ_ORDER == 3
            Eigen::Matrix<double, 3, 2> gradByHeadPV_post;
            Eigen::Matrix<double, 3, 2> gradByTailPV_post;
            obj.minco_post_.propogateGrad(obj.partialGradByCoeffs_post_, obj.partialGradByTimes_post_,
                                           obj.gradByPoints_post_, obj.gradByTimes_post_,
                                           gradByHeadPV_post, gradByTailPV_post);
            grad_collision_pos_from_post = gradByHeadPV_post.col(0);
#elif TRAJ_ORDER == 5
            Eigen::Matrix3d gradByHeadPVA_post;
            Eigen::Matrix3d gradByTailPVA_post;
            obj.minco_post_.propogateGrad(obj.partialGradByCoeffs_post_, obj.partialGradByTimes_post_,
                                           obj.gradByPoints_post_, obj.gradByTimes_post_,
                                           gradByHeadPVA_post, gradByTailPVA_post);
            grad_collision_pos_from_post = gradByHeadPVA_post.col(0);
#elif TRAJ_ORDER == 7
            Eigen::Matrix<double, 3, 4> gradByHeadPVAJ_post;
            Eigen::Matrix<double, 3, 4> gradByTailPVAJ_post;
            obj.minco_post_.propogateGrad(obj.partialGradByCoeffs_post_, obj.partialGradByTimes_post_,
                                           obj.gradByPoints_post_, obj.gradByTimes_post_,
                                           gradByHeadPVAJ_post, gradByTailPVAJ_post);
            grad_collision_pos_from_post = gradByHeadPVAJ_post.col(0);
#endif
            backwardGradT(tau_post, obj.gradByTimes_post_, grad_tau_post);
            backwardGradP(xi_post, obj.vPolyIdx_post_, obj.vPolytopes_post_, obj.gradByPoints_post_, grad_xi_post);
            normRetrictionLayer(xi_post, obj.vPolyIdx_post_, obj.vPolytopes_post_, cost, grad_xi_post);

            // TRT soft-constraint removed: TRT will still be used elsewhere to compute
            // a post-collision velocity for initializing the post segment, but it
            // will not participate as a penalty term inside the joint optimization.
            // 保持碰撞/末端松弛变量的梯度（不要在此处清零），让之前计算的梯度回传给优化器。

            // collision_u gradient (chain rule): dJ/du = (du_constrained/du)^T * (dp/du_constrained)^T * dJ/dp
            // where p = reference_point + M_dir * u_constrained
            {
                Eigen::Vector3d grad_p = grad_collision_pos_from_pre + grad_collision_pos_from_post;
                if (!grad_p.allFinite()) {
                    return INFINITY;
                }
                const double grad_u_con = M_dir.dot(grad_p);
                const double grad_u = ducon_du * grad_u_con;
                if (!std::isfinite(grad_u)) return INFINITY;
                grad_collision_u[0] = grad_u;
            }

            if (!grad.allFinite()) {
                ROS_ERROR("costFunctionalJoint: gradient contains NaN/Inf");
                return INFINITY;
            }

            return cost;
        }

        /**
         * @brief 执行联合优化
         * @param traj_pre   输出：pre 段轨迹
         * @param traj_post  输出：post 段轨迹
         * @param relCostTol 相对代价容差
         * @return 最终代价（INFINITY 表示失败）
         */
        inline double optimizeJoint(Trajectory<TRAJ_ORDER>& traj_pre,
                                     Trajectory<TRAJ_ORDER>& traj_post,
                                     const double& relCostTol)
        {
            const int dimTau_pre = temporalDim_pre_;
            const int dimXi_pre = spatialDim_pre_;
            const int dimTau_post = temporalDim_post_;
            const int dimXi_post = spatialDim_post_;
            const int dimPreVel = 3;
            const int dimCollision = 1;
            const int totalDim = dimTau_pre + dimXi_pre + dimTau_post + dimXi_post + dimPreVel + dimCollision;

            Eigen::VectorXd x(totalDim);

            // 初始化 pre 段
            setInitial(shortPath_pre_, allocSpeed, pieceIdx_pre_, points_pre_, times_pre_);

            // Override pre initial points from upstream polyline if available
            if (initial_guess_path_pre_.cols() >= 2 && pieceN_pre_ >= 2)
            {
                Eigen::Matrix3Xd path = initial_guess_path_pre_;
                path.col(0) = headPVAJ_pre_.col(0);
                path.col(path.cols() - 1) = tailPVAJ_pre_.col(0);
                Eigen::Matrix3Xd sampled = resamplePolylineByArcLength(path, pieceN_pre_ + 1);
                if (sampled.cols() == pieceN_pre_ + 1)
                {
                    Eigen::Matrix3Xd pts = sampled.middleCols(1, pieceN_pre_ - 1);
                    // ensure inside expected polytopes/intersections
                    auto getHPre = [&](int vpoly_idx) -> PolyhedronH {
                        if (vpoly_idx < 0) return PolyhedronH(6, 0);
                        if ((vpoly_idx % 2) == 0) {
                            int i = vpoly_idx / 2;
                            if (i < 0 || i >= (int)hPolytopes_pre_.size()) return PolyhedronH(6, 0);
                            return hPolytopes_pre_[i];
                        }
                        int i = (vpoly_idx - 1) / 2;
                        if (i < 0 || (i + 1) >= (int)hPolytopes_pre_.size()) return PolyhedronH(6, 0);
                        const auto &a = hPolytopes_pre_[i];
                        const auto &b = hPolytopes_pre_[i + 1];
                        PolyhedronH inter(6, a.cols() + b.cols());
                        inter.leftCols(a.cols()) = a;
                        inter.rightCols(b.cols()) = b;
                        return inter;
                    };
                    for (int j = 0; j < pts.cols(); ++j)
                    {
                        PolyhedronH h = getHPre(vPolyIdx_pre_(j));
                        if (h.cols() > 0 && !isPointInHPoly(h, pts.col(j), 1.0e-6))
                        {
                            Eigen::Vector3d interior;
                            if (geoutils::findInterior(h, interior))
                            {
                                pts.col(j) = interior;
                            }
                        }
                    }
                    points_pre_ = pts;
                    ROS_INFO("COLLISION_GCOPTER(JOINT): using provided pre initial guess path to initialize points");
                }
            }

            // Override pre initial times from upstream if available
            if (initial_segment_times_pre_.size() > 0)
            {
                if (initial_segment_times_pre_.size() == pieceN_pre_)
                {
                    times_pre_ = initial_segment_times_pre_;
                }
                else
                {
                    const double provided_total = initial_segment_times_pre_.sum();
                    if (provided_total > 1e-9)
                    {
                        Eigen::VectorXd pieceLengths(pieceN_pre_);
                        int idx = 0;
                        Eigen::Matrix3Xd deltas = shortPath_pre_.rightCols(polyN_pre_) - shortPath_pre_.leftCols(polyN_pre_);
                        for (int i = 0; i < polyN_pre_; ++i)
                        {
                            double seg_len = deltas.col(i).norm();
                            int k = pieceIdx_pre_(i);
                            if (k <= 0) k = 1;
                            for (int j = 0; j < k; ++j)
                            {
                                pieceLengths(idx++) = seg_len / double(k);
                            }
                        }
                        double len_sum = pieceLengths.sum();
                        if (len_sum <= 1e-9) len_sum = 1.0;
                        times_pre_ = provided_total * pieceLengths / len_sum;
                    }
                }
            }
            setResVector(headPVAJ_pre_.col(0), tailPVAJ_pre_.col(0), points_pre_, times_pre_, integralRes, resVector_pre_);
            Eigen::VectorXd tau_pre(dimTau_pre), xi_pre(dimXi_pre);
            backwardT(times_pre_, tau_pre);
            backwardP(points_pre_, vPolyIdx_pre_, vPolytopes_pre_, xi_pre);

            // 初始化 post 段
            setInitial(shortPath_post_, allocSpeed, pieceIdx_post_, points_post_, times_post_);

            // Override post initial points from upstream polyline if available
            if (initial_guess_path_post_.cols() >= 2 && pieceN_post_ >= 2)
            {
                Eigen::Matrix3Xd path = initial_guess_path_post_;
                path.col(0) = headPVAJ_post_.col(0);
                path.col(path.cols() - 1) = tailPVAJ_post_.col(0);
                Eigen::Matrix3Xd sampled = resamplePolylineByArcLength(path, pieceN_post_ + 1);
                if (sampled.cols() == pieceN_post_ + 1)
                {
                    Eigen::Matrix3Xd pts = sampled.middleCols(1, pieceN_post_ - 1);
                    auto getHPost = [&](int vpoly_idx) -> PolyhedronH {
                        if (vpoly_idx < 0) return PolyhedronH(6, 0);
                        if ((vpoly_idx % 2) == 0) {
                            int i = vpoly_idx / 2;
                            if (i < 0 || i >= (int)hPolytopes_post_.size()) return PolyhedronH(6, 0);
                            return hPolytopes_post_[i];
                        }
                        int i = (vpoly_idx - 1) / 2;
                        if (i < 0 || (i + 1) >= (int)hPolytopes_post_.size()) return PolyhedronH(6, 0);
                        const auto &a = hPolytopes_post_[i];
                        const auto &b = hPolytopes_post_[i + 1];
                        PolyhedronH inter(6, a.cols() + b.cols());
                        inter.leftCols(a.cols()) = a;
                        inter.rightCols(b.cols()) = b;
                        return inter;
                    };
                    for (int j = 0; j < pts.cols(); ++j)
                    {
                        PolyhedronH h = getHPost(vPolyIdx_post_(j));
                        if (h.cols() > 0 && !isPointInHPoly(h, pts.col(j), 1.0e-6))
                        {
                            Eigen::Vector3d interior;
                            if (geoutils::findInterior(h, interior))
                            {
                                pts.col(j) = interior;
                            }
                        }
                    }
                    points_post_ = pts;
                    ROS_INFO("COLLISION_GCOPTER(JOINT): using provided post initial guess path to initialize points");
                }
            }

            // Override post initial times from upstream if available
            if (initial_segment_times_post_.size() > 0)
            {
                if (initial_segment_times_post_.size() == pieceN_post_)
                {
                    times_post_ = initial_segment_times_post_;
                }
                else
                {
                    const double provided_total = initial_segment_times_post_.sum();
                    if (provided_total > 1e-9)
                    {
                        Eigen::VectorXd pieceLengths(pieceN_post_);
                        int idx = 0;
                        Eigen::Matrix3Xd deltas = shortPath_post_.rightCols(polyN_post_) - shortPath_post_.leftCols(polyN_post_);
                        for (int i = 0; i < polyN_post_; ++i)
                        {
                            double seg_len = deltas.col(i).norm();
                            int k = pieceIdx_post_(i);
                            if (k <= 0) k = 1;
                            for (int j = 0; j < k; ++j)
                            {
                                pieceLengths(idx++) = seg_len / double(k);
                            }
                        }
                        double len_sum = pieceLengths.sum();
                        if (len_sum <= 1e-9) len_sum = 1.0;
                        times_post_ = provided_total * pieceLengths / len_sum;
                    }
                }
            }
            setResVector(headPVAJ_post_.col(0), tailPVAJ_post_.col(0), points_post_, times_post_, integralRes, resVector_post_);
            Eigen::VectorXd tau_post(dimTau_post), xi_post(dimXi_post);
            backwardT(times_post_, tau_post);
            backwardP(points_post_, vPolyIdx_post_, vPolytopes_post_, xi_post);

            // 组装 x
            int offset = 0;
            x.segment(offset, dimTau_pre) = tau_pre; offset += dimTau_pre;
            x.segment(offset, dimXi_pre) = xi_pre; offset += dimXi_pre;
            x.segment(offset, dimTau_post) = tau_post; offset += dimTau_post;
            x.segment(offset, dimXi_post) = xi_post; offset += dimXi_post;
            x.segment(offset, dimPreVel) = tailPVAJ_pre_.col(1); offset += dimPreVel;
            x.segment(offset, dimCollision).setZero(); // collision_u

            // L-BFGS 参数
            lbfgs::lbfgs_parameter_t params;
            params.mem_size = 256;
            params.past = 3;
            params.min_step = 1.0e-32;
            params.g_epsilon = 0.0;
            params.delta = relCostTol;

            double minCost;
            int ret = lbfgs::lbfgs_optimize(x, minCost,
                                             &COLLISION_GCOPTER::costFunctionalJoint,
                                             nullptr, nullptr, this, params);

            ROS_INFO("optimizeJoint: L-BFGS returned %d, cost=%.6f", ret, minCost);

            if (ret >= 0) {
                // 解析结果
                offset = 0;
                Eigen::Map<const Eigen::VectorXd> tau_pre_opt(x.data() + offset, dimTau_pre); offset += dimTau_pre;
                Eigen::Map<const Eigen::VectorXd> xi_pre_opt(x.data() + offset, dimXi_pre); offset += dimXi_pre;
                Eigen::Map<const Eigen::VectorXd> tau_post_opt(x.data() + offset, dimTau_post); offset += dimTau_post;
                Eigen::Map<const Eigen::VectorXd> xi_post_opt(x.data() + offset, dimXi_post);

                // 解析 v_pre_end
                offset = dimTau_pre + dimXi_pre + dimTau_post + dimXi_post;
                Eigen::Map<const Eigen::VectorXd> v_pre_end_opt(x.data() + offset, dimPreVel);
                tailPVAJ_pre_.col(1) = v_pre_end_opt.head<3>();
                tailPVAJ_pre_.col(3).setZero(); // ensure jerk = 0 after extracting optimized pre-end velocity

                // 解析 collision_u，并把优化后的碰撞点位置写回到 pre tail / post head
                offset += dimPreVel;
                const double u_param = x[offset];
                if (!collision_events_.empty()) {
                    const auto &event = collision_events_[0];
                    const Eigen::Vector3d world_z(0.0, 0.0, 1.0);
                    const Eigen::Vector3d t0 = event.rotation_matrix * event.basis_matrix.col(0);
                    const Eigen::Vector3d t1 = event.rotation_matrix * event.basis_matrix.col(1);
                    const int t_idx = (std::abs(t0.dot(world_z)) <= std::abs(t1.dot(world_z))) ? 0 : 1;
                    const Eigen::Vector3d M_dir = (event.rotation_matrix * event.basis_matrix.col(t_idx)) * event.max_position_offset;

                    double u_constrained = u_param;
                    if (u_param > 1.0) u_constrained = 1.0;
                    else if (u_param < -1.0) u_constrained = -1.0;

                    const Eigen::Vector3d collision_pt_eval = event.reference_point + M_dir * u_constrained;
                    if (collision_pt_eval.allFinite()) {
                        tailPVAJ_pre_.col(0) = collision_pt_eval;
                        headPVAJ_post_.col(0) = collision_pt_eval;
                    }
                }

                forwardT(tau_pre_opt, times_pre_);
                forwardP(xi_pre_opt, vPolyIdx_pre_, vPolytopes_pre_, points_pre_);

#if TRAJ_ORDER == 3
                collision_minco.setConditions(headPVAJ_pre_.leftCols(2), tailPVAJ_pre_.leftCols(2), pieceN_pre_);
#elif TRAJ_ORDER == 5
                collision_minco.setConditions(headPVAJ_pre_.leftCols(3), tailPVAJ_pre_.leftCols(3), pieceN_pre_);
#elif TRAJ_ORDER == 7
                collision_minco.setConditions(headPVAJ_pre_, tailPVAJ_pre_, pieceN_pre_);
#endif
                collision_minco.setParameters(points_pre_, times_pre_);
                collision_minco.getTrajectory(traj_pre);

                forwardT(tau_post_opt, times_post_);
                forwardP(xi_post_opt, vPolyIdx_post_, vPolytopes_post_, points_post_);

#if TRAJ_ORDER == 3
                minco_post_.setConditions(headPVAJ_post_.leftCols(2), tailPVAJ_post_.leftCols(2), pieceN_post_);
#elif TRAJ_ORDER == 5
                minco_post_.setConditions(headPVAJ_post_.leftCols(3), tailPVAJ_post_.leftCols(3), pieceN_post_);
#elif TRAJ_ORDER == 7
                minco_post_.setConditions(headPVAJ_post_, tailPVAJ_post_, pieceN_post_);
#endif
                minco_post_.setParameters(points_post_, times_post_);
                minco_post_.getTrajectory(traj_post);

                return minCost;
            } else {
                traj_pre.clear();
                traj_post.clear();
                return INFINITY;
            }
        }

        /**
         * @brief 联合优化版本的多段轨迹优化（单碰撞专用）
         * 替代串行的 optimizeMultiSegmentTrajectory，实现 pre+post 联合优化
         */
        inline bool optimizeCollisionTrajectoryJoint(
            const TrajectorySegment& pre_segment,
            const TrajectorySegment& post_segment,
            const double& relCostTol,
            const double& timeWeight,
            const double& lengthPerPiece,
            const double& smoothingFactor,
            const int& integralResolution,
            const Eigen::VectorXd& magnitudeBounds,
            const Eigen::VectorXd& penaltyWeights,
            const bool& isDebug,
            Visualizer& visual,
            ros::Publisher& optPuber,
            const double& friction,
            const double& damping_ratio,
            const double& max_collision_velocity,
            const double& trt_velocity_weight,
            const double& position_constraint_weight,
            Trajectory<TRAJ_ORDER>& traj_pre_out,
            Trajectory<TRAJ_ORDER>& traj_post_out)
        {
            if (!pre_segment.is_collision_segment) {
                ROS_ERROR("optimizeCollisionTrajectoryJoint: pre_segment must be a collision segment");
                return false;
            }

            // 提取碰撞点
            Eigen::Vector3d collision_pt = pre_segment.collision_event.collision_point;

            // 设置联合优化
            if (!setupJointOptimization(
                    pre_segment.corridor,
                    post_segment.corridor,
                    pre_segment.start_PVAJ,
                    collision_pt,
                    post_segment.end_PVAJ,
                    pre_segment.collision_event,
                    pre_segment.initial_segment_times,
                    pre_segment.initial_path_waypoints,
                    post_segment.initial_segment_times,
                    post_segment.initial_path_waypoints,
                    timeWeight,
                    lengthPerPiece,
                    smoothingFactor,
                    integralResolution,
                    magnitudeBounds,
                    penaltyWeights,
                    friction,
                    damping_ratio,
                    max_collision_velocity,
                    trt_velocity_weight,
                    position_constraint_weight,
                    isDebug,
                    visual,
                    optPuber)) {
                return false;
            }

            // 方案B：外部迭代，碰撞前速度变为可优化，碰撞后速度通过TRT反馈确定
            const int max_iterations = 1;  // 最大迭代次数（外层只迭代一次）
            const double convergence_threshold = 0.05;  // 速度收敛阈值
            const double relaxation_factor = 0.7;  // 松弛因子，防止震荡
            
            Eigen::Vector3d pre_collision_vel_prev = tailPVAJ_pre_.col(1);
            Eigen::Vector3d post_collision_vel_prev = headPVAJ_post_.col(1);
            
            ROS_INFO("=== Starting Iterative Joint Optimization (Plan-B) ===");
            ROS_INFO("Initial pre_vel: (%.3f,%.3f,%.3f), post_vel: (%.3f,%.3f,%.3f)",
                     pre_collision_vel_prev.x(), pre_collision_vel_prev.y(), pre_collision_vel_prev.z(),
                     post_collision_vel_prev.x(), post_collision_vel_prev.y(), post_collision_vel_prev.z());
            
            bool converged = false;
            double final_cost = INFINITY;
            
            for (int iter = 0; iter < max_iterations; iter++) {
                ROS_INFO("--- Iteration %d ---", iter + 1);
                
                // 执行联合优化（pre速度可优化，post速度固定）
                double cost = optimizeJoint(traj_pre_out, traj_post_out, relCostTol);
                if (cost >= INFINITY) {
                    ROS_ERROR("optimizeCollisionTrajectoryJoint: optimization failed at iteration %d", iter + 1);
                    return false;
                }
                final_cost = cost;
                
                // 获取优化后的碰撞前速度
                double T_pre = traj_pre_out.getTotalDuration();
                Eigen::Vector3d v_pre_end_optimized = traj_pre_out.getVel(T_pre);
                
                // 使用TRT计算新的碰撞后速度
                const auto& evt = collision_events_[0];
                Eigen::Vector3d v_post_TRT = evt.calculateDynamicPostCollisionVelocity(
                    traj_pre_out, -1.0, friction_, damping_ratio_, evt.given_yaw);
                
                ROS_INFO("Iter %d: optimized pre_vel=(%.3f,%.3f,%.3f), TRT post_vel=(%.3f,%.3f,%.3f)",
                         iter + 1, v_pre_end_optimized.x(), v_pre_end_optimized.y(), v_pre_end_optimized.z(),
                         v_post_TRT.x(), v_post_TRT.y(), v_post_TRT.z());
                
                // 检查收敛性
                double pre_vel_change = (v_pre_end_optimized - pre_collision_vel_prev).norm();
                double post_vel_change = (v_post_TRT - post_collision_vel_prev).norm();
                
                ROS_INFO("Iter %d: pre_vel_change=%.4f, post_vel_change=%.4f", 
                         iter + 1, pre_vel_change, post_vel_change);
                
                if (pre_vel_change < convergence_threshold && post_vel_change < convergence_threshold) {
                    converged = true;
                    ROS_INFO("Converged at iteration %d!", iter + 1);
                    break;
                }
                
                // 更新边界条件（使用松弛因子防止震荡）
                Eigen::Vector3d new_pre_vel = pre_collision_vel_prev + 
                    relaxation_factor * (v_pre_end_optimized - pre_collision_vel_prev);
                Eigen::Vector3d new_post_vel = post_collision_vel_prev + 
                    relaxation_factor * (v_post_TRT - post_collision_vel_prev);
                
                // 速度幅值安全检查
                if (new_pre_vel.norm() > max_collision_velocity_ || new_post_vel.norm() > max_collision_velocity_) {
                    ROS_WARN("Iter %d: Velocity magnitude exceeds limit, normalizing...", iter + 1);
                    if (new_pre_vel.norm() > max_collision_velocity_) {
                        new_pre_vel = new_pre_vel.normalized() * max_collision_velocity_;
                    }
                    if (new_post_vel.norm() > max_collision_velocity_) {
                        new_post_vel = new_post_vel.normalized() * max_collision_velocity_;
                    }
                }
                
                // 更新边界条件为下一次迭代
                tailPVAJ_pre_.col(1) = new_pre_vel;
                headPVAJ_post_.col(1) = new_post_vel;
                
                // 重新设置MINCO边界条件
#if TRAJ_ORDER == 3
                collision_minco.setConditions(headPVAJ_pre_.leftCols(2), tailPVAJ_pre_.leftCols(2), pieceN_pre_);
                minco_post_.setConditions(headPVAJ_post_.leftCols(2), tailPVAJ_post_.leftCols(2), pieceN_post_);
#elif TRAJ_ORDER == 5
                collision_minco.setConditions(headPVAJ_pre_.leftCols(3), tailPVAJ_pre_.leftCols(3), pieceN_pre_);
                minco_post_.setConditions(headPVAJ_post_.leftCols(3), tailPVAJ_post_.leftCols(3), pieceN_post_);
#elif TRAJ_ORDER == 7
                collision_minco.setConditions(headPVAJ_pre_, tailPVAJ_pre_, pieceN_pre_);
                minco_post_.setConditions(headPVAJ_post_, tailPVAJ_post_, pieceN_post_);
#endif
                
                // 更新迭代变量
                pre_collision_vel_prev = new_pre_vel;
                post_collision_vel_prev = new_post_vel;
            }
            
            if (!converged) {
                ROS_WARN("Iterative optimization did not converge within %d iterations", max_iterations);
            }
            
            // 打印最终结果
            double T_pre = traj_pre_out.getTotalDuration();
            double T_post = traj_post_out.getTotalDuration();
            Eigen::Vector3d v_pre_end = traj_pre_out.getVel(T_pre);
            Eigen::Vector3d v_post_start = traj_post_out.getVel(0.0);

            const auto& evt = collision_events_[0];
            Eigen::Vector3d v_post_TRT = evt.calculateDynamicPostCollisionVelocity(
                traj_pre_out, -1.0, friction_, damping_ratio_, evt.given_yaw);

            ROS_INFO("=== Final Joint Optimization Result (Plan-B) ===");
            ROS_INFO("Converged: %s, Final cost: %.6f", converged ? "Yes" : "No", final_cost);
            ROS_INFO("Pre segment: duration=%.3f, end_vel=(%.3f,%.3f,%.3f)",
                     T_pre, v_pre_end.x(), v_pre_end.y(), v_pre_end.z());
            ROS_INFO("=== Final Joint Optimization Result (Plan-B) ===");
            ROS_INFO("Converged: %s, Final cost: %.6f", converged ? "Yes" : "No", final_cost);
            ROS_INFO("Pre segment: duration=%.3f, end_vel=(%.3f,%.3f,%.3f)",
                     T_pre, v_pre_end.x(), v_pre_end.y(), v_pre_end.z());
            ROS_INFO("Post segment: duration=%.3f, start_vel=(%.3f,%.3f,%.3f)",
                     T_post, v_post_start.x(), v_post_start.y(), v_post_start.z());
            ROS_INFO("TRT predicted post_vel: (%.3f,%.3f,%.3f)",
                     v_post_TRT.x(), v_post_TRT.y(), v_post_TRT.z());
            ROS_INFO("Velocity match error: %.6f m/s", (v_post_start - v_post_TRT).norm());

            return true;
        }
    };
} // namespace collision_gcopter


#endif
