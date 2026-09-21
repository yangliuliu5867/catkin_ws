#ifndef COLLISION_RANSAC_HPP
#define COLLISION_RANSAC_HPP

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

/**
 * @brief 用于kinodynamic RRT*碰撞检测的RANSAC平面拟合类
 * 
 * 该类专门用于在碰撞点附近拟合碰撞平面，支持从点云中提取局部平面信息
 * 用于计算碰撞法向量，进而计算反弹速度
 */
class CollisionRANSAC
{
public:
    /**
     * @brief 3D点结构体，兼容Eigen和PCL
     */
    struct Point3D
    {
        double x, y, z;
        
        Point3D() : x(0), y(0), z(0) {}
        Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
        Point3D(const Eigen::Vector3d& vec) : x(vec.x()), y(vec.y()), z(vec.z()) {}
        Point3D(const pcl::PointXYZ& pt) : x(pt.x), y(pt.y), z(pt.z) {}
        
        // 转换为Eigen向量
        Eigen::Vector3d to_eigen() const {
            return Eigen::Vector3d(x, y, z);
        }
        
        // 向量运算
        Point3D operator+(const Point3D& other) const {
            return Point3D(x + other.x, y + other.y, z + other.z);
        }
        
        Point3D operator-(const Point3D& other) const {
            return Point3D(x - other.x, y - other.y, z - other.z);
        }
        
        Point3D operator*(double scalar) const {
            return Point3D(x * scalar, y * scalar, z * scalar);
        }
        
        // 内积
        double dot(const Point3D& other) const {
            return x * other.x + y * other.y + z * other.z;
        }
        
        // 外积
        Point3D cross(const Point3D& other) const {
            return Point3D(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }
        
        // 归一化
        void normalize() {
            double norm = sqrt(x * x + y * y + z * z);
            if (norm > 1e-8) {
                x /= norm;
                y /= norm;
                z /= norm;
            }
        }
        
        // 获取模长
        double norm() const {
            return sqrt(x * x + y * y + z * z);
        }
    };
    
    /**
     * @brief 平面结构体，存储平面方程参数
     */
    struct Plane
    {
        Point3D point;      // 平面上一点
        Point3D normal;     // 平面法向量（单位向量）
        double d;           // 平面方程参数d，满足 ax + by + cz + d = 0
        
        Plane() : d(0.0) {}
        
        // 根据点和法向量构造平面
        Plane(const Point3D& pt, const Point3D& norm) : point(pt), normal(norm) {
            normal.normalize();
            d = -(normal.x * point.x + normal.y * point.y + normal.z * point.z);
        }
        
        // 计算点到平面的距离
        double distance_to_point(const Point3D& pt) const {
            return std::abs(normal.x * pt.x + normal.y * pt.y + normal.z * pt.z + d) / normal.norm();
        }
    };

    /**
     * @brief 平面质量评估结构体
     */
    struct PlaneQuality
    {
        int inlier_count;               // 内点数量
        double inlier_ratio;            // 内点比例
        double average_residual;        // 平均残差
        double max_residual;            // 最大残差
        double normal_axis_alignment;   // 法向量与主轴的对齐度 [0,1]
        bool is_axis_aligned;           // 是否轴对齐
        bool is_single_plane;           // 是否为单一平面（非拐角）
        double overall_score;           // 综合质量评分 [0,1]
        
        PlaneQuality() : inlier_count(0), inlier_ratio(0.0), average_residual(0.0), 
                        max_residual(0.0), normal_axis_alignment(0.0), 
                        is_axis_aligned(false), is_single_plane(false), overall_score(0.0) {}
    };

private:
    std::vector<Point3D> points_;           // 输入点云
    Plane best_plane_;                      // 最佳拟合平面
    std::vector<int> inlier_indices_;       // 内点索引
    PlaneQuality plane_quality_;            // 平面质量评估
    
    // RANSAC参数
    double distance_threshold_;             // 距离阈值
    int max_iterations_;                    // 最大迭代次数
    double confidence_;                     // 置信度
    
    // 墙壁质量判断参数
    int min_inlier_count_;                  // 最小内点数量
    double min_inlier_ratio_;               // 最小内点比例
    double max_average_residual_;           // 最大平均残差
    double max_residual_threshold_;         // 最大残差阈值
    double axis_alignment_threshold_;       // 轴对齐阈值（度）
    
    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dis_;

public:
    /**
     * @brief 构造函数
     * @param distance_threshold RANSAC距离阈值
     * @param max_iterations 最大迭代次数
     * @param confidence 置信度 (0-1)
     */
    CollisionRANSAC(double distance_threshold = 0.05, 
                   int max_iterations = 1000, 
                   double confidence = 0.99)
        : distance_threshold_(distance_threshold)
        , max_iterations_(max_iterations)
        , confidence_(confidence)
        , min_inlier_count_(20)              // 至少20个内点
        , min_inlier_ratio_(0.3)             // 至少30%的点是内点
        , max_average_residual_(0.02)        // 平均残差不超过2cm
        , max_residual_threshold_(0.1)       // 最大残差不超过10cm
        , axis_alignment_threshold_(20.0)    // 与主轴夹角不超过20度
        , gen_(rd_())
        , dis_(0.0, 1.0)
    {
    }
    
    /**
     * @brief 设置RANSAC参数
     */
    void set_parameters(double distance_threshold, int max_iterations, double confidence = 0.99) {
        distance_threshold_ = distance_threshold;
        max_iterations_ = max_iterations;
        confidence_ = confidence;
    }
    
    /**
     * @brief 设置墙壁质量判断参数
     */
    void set_quality_parameters(int min_inlier_count = 20,
                               double min_inlier_ratio = 0.3,
                               double max_average_residual = 0.02,
                               double max_residual = 0.1,
                               double axis_alignment_threshold = 20.0) {
        min_inlier_count_ = min_inlier_count;
        min_inlier_ratio_ = min_inlier_ratio;
        max_average_residual_ = max_average_residual;
        max_residual_threshold_ = max_residual;
        axis_alignment_threshold_ = axis_alignment_threshold;
    }
    
    /**
     * @brief 从PCL点云中加载数据
     * @param cloud PCL点云
     * @param collision_point 碰撞点
     * @param search_radius 搜索半径
     */
    bool load_from_pcl_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const Eigen::Vector3d& collision_point,
                             double search_radius) {
        points_.clear();
        
        if (!cloud || cloud->empty()) {
            ROS_WARN("CollisionRANSAC: Empty point cloud provided");
            return false;
        }
        
        // 使用KDTree进行半径搜索
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        
        pcl::PointXYZ search_point;
        search_point.x = collision_point.x();
        search_point.y = collision_point.y();
        search_point.z = collision_point.z();
        
        std::vector<int> indices;
        std::vector<float> distances;
        
        int found = kdtree.radiusSearch(search_point, search_radius, indices, distances);
        
        if (found < 3) {
            ROS_WARN("CollisionRANSAC: Not enough points found near collision point (found: %d)", found);
            return false;
        }
        
        // 转换为内部点格式
        for (int idx : indices) {
            points_.emplace_back(cloud->points[idx]);
        }
        
        return true;
    }
    
    /**
     * @brief 从Eigen向量列表加载数据
     */
    void load_from_eigen_points(const std::vector<Eigen::Vector3d>& eigen_points) {
        points_.clear();
        points_.reserve(eigen_points.size());
        
        for (const auto& pt : eigen_points) {
            points_.emplace_back(pt);
        }
    }
    
    /**
     * @brief 执行RANSAC平面拟合
     * @return 是否成功拟合平面
     */
    bool fit_plane() {
        if (points_.size() < 3) {
            ROS_ERROR("CollisionRANSAC: Need at least 3 points for plane fitting");
            return false;
        }
        
        int best_inlier_count = 0;
        Plane best_plane;
        std::vector<int> best_inliers;
        
        // 动态调整迭代次数
        int adaptive_iterations = max_iterations_;
        
        for (int iter = 0; iter < adaptive_iterations; ++iter) {
            // 随机选择3个不重复的点
            std::vector<int> sample_indices = random_sample(3);
            if (sample_indices.size() != 3) {
                continue;
            }
            
            // 检查三点是否共线
            if (are_collinear(points_[sample_indices[0]], 
                             points_[sample_indices[1]], 
                             points_[sample_indices[2]])) {
                continue;
            }
            
            // 拟合平面
            Plane candidate_plane;
            if (!fit_plane_from_three_points(points_[sample_indices[0]],
                                            points_[sample_indices[1]],
                                            points_[sample_indices[2]],
                                            candidate_plane)) {
                continue;
            }
            
            // 计算内点数量
            std::vector<int> inliers;
            for (size_t i = 0; i < points_.size(); ++i) {
                if (candidate_plane.distance_to_point(points_[i]) < distance_threshold_) {
                    inliers.push_back(i);
                }
            }
            
            // 更新最佳模型
            if (static_cast<int>(inliers.size()) > best_inlier_count) {
                best_inlier_count = static_cast<int>(inliers.size());
                best_plane = candidate_plane;
                best_inliers = inliers;
                
                // 动态调整迭代次数
                double inlier_ratio = static_cast<double>(best_inlier_count) / points_.size();
                if (inlier_ratio > 0.1) { // 至少10%的点是内点
                    int new_iterations = static_cast<int>(
                        std::log(1.0 - confidence_) / std::log(1.0 - std::pow(inlier_ratio, 3))
                    );
                    adaptive_iterations = std::min(adaptive_iterations, new_iterations);
                }
                
                // 如果内点比例足够高，提前退出
                if (inlier_ratio > 0.8) {
                    break;
                }
            }
        }
        
        if (best_inlier_count < 3) {
            ROS_WARN("CollisionRANSAC: Failed to find valid plane (best inliers: %d)", best_inlier_count);
            return false;
        }
        
        // 保存结果
        best_plane_ = best_plane;
        inlier_indices_ = best_inliers;
        
        // === 新增：评估平面质量 ===
        evaluate_plane_quality();
        
        return true;
    }
    
    /**
     * @brief 检查拟合的平面是否为高质量的单一墙面
     * @return 是否为合格的墙面
     */
    bool is_valid_wall_plane() const {
        return plane_quality_.is_single_plane && 
               plane_quality_.inlier_count >= min_inlier_count_ &&
               plane_quality_.inlier_ratio >= min_inlier_ratio_ &&
               plane_quality_.average_residual <= max_average_residual_ &&
               plane_quality_.overall_score >= 0.6;  // 综合评分阈值
    }
    
    /**
     * @brief 获取拟合的平面
     */
    const Plane& get_plane() const {
        return best_plane_;
    }
    
    /**
     * @brief 获取平面法向量（Eigen格式）
     */
    Eigen::Vector3d get_normal_eigen() const {
        return best_plane_.normal.to_eigen();
    }
    
    /**
     * @brief 获取平面方程参数d
     */
    double get_plane_d() const {
        return best_plane_.d;
    }
    
    /**
     * @brief 获取内点索引
     */
    const std::vector<int>& get_inlier_indices() const {
        return inlier_indices_;
    }
    
    /**
     * @brief 获取内点数量
     */
    int get_inlier_count() const {
        return static_cast<int>(inlier_indices_.size());
    }
    
    /**
     * @brief 获取内点比例
     */
    double get_inlier_ratio() const {
        if (points_.empty()) return 0.0;
        return static_cast<double>(inlier_indices_.size()) / points_.size();
    }
    
    /**
     * @brief 获取平均残差
     */
    double get_average_residual() const {
        return plane_quality_.average_residual;
    }
    
    /**
     * @brief 获取最大残差
     */
    double get_max_residual() const {
        return plane_quality_.max_residual;
    }
    
    /**
     * @brief 获取平面质量评估结果
     */
    const PlaneQuality& get_plane_quality() const {
        return plane_quality_;
    }

private:
    /**
     * @brief 评估拟合平面的质量
     */
    void evaluate_plane_quality() {
        plane_quality_ = PlaneQuality();
        
        if (inlier_indices_.empty()) {
            return;
        }
        
        // 1. 基本统计信息
        plane_quality_.inlier_count = static_cast<int>(inlier_indices_.size());
        plane_quality_.inlier_ratio = static_cast<double>(inlier_indices_.size()) / points_.size();
        
        // 2. 计算残差统计
        double sum_residual = 0.0;
        double max_residual = 0.0;
        
        for (int idx : inlier_indices_) {
            double residual = best_plane_.distance_to_point(points_[idx]);
            sum_residual += residual;
            max_residual = std::max(max_residual, residual);
        }
        
        plane_quality_.average_residual = sum_residual / inlier_indices_.size();
        plane_quality_.max_residual = max_residual;
        
        // 3. 评估法向量的轴对齐度
        Eigen::Vector3d normal = best_plane_.normal.to_eigen();
        double angle_x = std::acos(std::abs(normal.x())) * 180.0 / M_PI;
        double angle_y = std::acos(std::abs(normal.y())) * 180.0 / M_PI;
        double angle_z = std::acos(std::abs(normal.z())) * 180.0 / M_PI;
        
        double min_axis_angle = std::min({angle_x, angle_y, angle_z});
        plane_quality_.normal_axis_alignment = 1.0 - (min_axis_angle / 90.0);  // [0,1]
        plane_quality_.is_axis_aligned = (min_axis_angle <= axis_alignment_threshold_);
        
        // 4. 判断是否为单一平面（非拐角）
        evaluate_single_plane_property();
        
        // 5. 计算综合质量评分
        calculate_overall_score();
    }
    
    /**
     * @brief 评估是否为单一平面（检测拐角、边缘等复杂几何）
     */
    void evaluate_single_plane_property() {
        // 方法1：检查非内点的分布
        // 如果大量非内点集中在某个方向，可能是拐角
        
        std::vector<int> outlier_indices;
        for (size_t i = 0; i < points_.size(); ++i) {
            bool is_inlier = std::find(inlier_indices_.begin(), inlier_indices_.end(), i) != inlier_indices_.end();
            if (!is_inlier) {
                outlier_indices.push_back(i);
            }
        }
        
        // 如果外点太多，可能不是单一平面
        double outlier_ratio = static_cast<double>(outlier_indices.size()) / points_.size();
        ROS_DEBUG("Outlier ratio: %.1f%% (threshold: 75%%)", outlier_ratio * 100.0);
        if (outlier_ratio > 0.65) {  // 超过65%是外点才认为不是单一平面
            plane_quality_.is_single_plane = false;
            ROS_DEBUG("Too many outliers (%.1f%%), likely not a single plane", outlier_ratio * 100.0);
            return;
        }
        
        // 方法2：检查内点的空间分布是否集中 (放宽要求)
        // 计算内点的主成分分析（简化版）
        bool spatial_ok = check_inlier_spatial_distribution();
        ROS_DEBUG("Spatial distribution check: %s", spatial_ok ? "PASS" : "FAIL");
        if (!spatial_ok) {
            // 不要直接拒绝，而是降低分数
            ROS_DEBUG("Inliers have poor spatial distribution, but continuing with reduced confidence");
            // plane_quality_.is_single_plane = false;
            // return;
        }
        
        // 方法3：多平面检测 (只有在外点足够多时才执行)
        // 尝试对外点再次拟合平面，如果成功且与主平面正交，则是拐角
        bool has_secondary_plane = false;
        if (outlier_indices.size() >= 15) {  // 至少15个外点才尝试二次拟合
            has_secondary_plane = detect_secondary_plane(outlier_indices);
            ROS_DEBUG("Secondary plane detection: %s", has_secondary_plane ? "DETECTED" : "NOT_DETECTED");
        }
        
        // 综合判断：只有在检测到明显的第二平面时才拒绝
        if (has_secondary_plane) {
            plane_quality_.is_single_plane = false;
            ROS_DEBUG("Detected secondary plane, likely corner structure");
            return;
        }
        
        plane_quality_.is_single_plane = true;
        ROS_DEBUG("Passed single plane tests");
    }
    
    /**
     * @brief 检查内点的空间分布集中度
     */
    bool check_inlier_spatial_distribution() {
        if (inlier_indices_.size() < 5) return false;
        
        // 计算内点的质心
        Point3D centroid(0, 0, 0);
        for (int idx : inlier_indices_) {
            centroid = centroid + points_[idx];
        }
        centroid = centroid * (1.0 / inlier_indices_.size());
        
        // 计算内点到质心的平均距离
        double avg_distance = 0.0;
        for (int idx : inlier_indices_) {
            avg_distance += (points_[idx] - centroid).norm();
        }
        avg_distance /= inlier_indices_.size();
        
        // 如果内点过于分散，可能不是单一平面 (放宽阈值从0.5m到0.8m)
        const double max_spread = 0.8;  // 最大分布半径0.8m
        ROS_DEBUG("Inlier spatial spread: avg_dist=%.3f (threshold=%.3f)", avg_distance, max_spread);
        if (avg_distance > max_spread) {
            ROS_DEBUG("Inliers too spread out (avg_dist=%.3f > %.3f)", avg_distance, max_spread);
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 检测外点中是否存在第二个平面（拐角检测）
     */
    bool detect_secondary_plane(const std::vector<int>& outlier_indices) {
        if (outlier_indices.size() < 8) return false;  // 至少需要8个外点
        
        // 对外点尝试简单的平面拟合
        // 随机选择3个外点拟合平面
        int attempts = 100;  // 增加尝试次数
        int best_secondary_inliers = 0;
        Plane best_secondary_plane;
        
        for (int attempt = 0; attempt < attempts; ++attempt) {
            if (outlier_indices.size() < 3) break;
            
            // 随机选择3个外点
            std::vector<int> sample(3);
            for (int i = 0; i < 3; ++i) {
                sample[i] = outlier_indices[static_cast<int>(dis_(gen_) * outlier_indices.size())];
            }
            
            // 检查是否重复
            if (sample[0] == sample[1] || sample[1] == sample[2] || sample[0] == sample[2]) {
                continue;
            }
            
            // 拟合平面
            Plane secondary_plane;
            if (!fit_plane_from_three_points(points_[sample[0]], points_[sample[1]], points_[sample[2]], secondary_plane)) {
                continue;
            }
            
            // 统计这个平面的内点数
            int secondary_inliers = 0;
            for (int idx : outlier_indices) {
                if (secondary_plane.distance_to_point(points_[idx]) < distance_threshold_) {
                    secondary_inliers++;
                }
            }
            
            // 更新最佳第二平面
            if (secondary_inliers > best_secondary_inliers) {
                best_secondary_inliers = secondary_inliers;
                best_secondary_plane = secondary_plane;
            }
        }
        
        ROS_DEBUG("Secondary plane detection: best_inliers=%d (min: 8)", best_secondary_inliers);
        
        // 只有在第二个平面有足够多的内点时才进行角度检查
        if (best_secondary_inliers >= 8) {  // 至少8个内点
            double dot_product = std::abs(best_plane_.normal.dot(best_secondary_plane.normal));
            double angle = std::acos(std::min(1.0, dot_product)) * 180.0 / M_PI;
            
            ROS_DEBUG("Secondary plane angle with main plane: %.1f deg", angle);
            
            // 检查是否接近正交 (更严格的角度范围)
            if (angle > 70.0 && angle < 110.0) {  // 接近90度±20度
                // 额外检查：第二平面的内点比例不能太小
                double secondary_ratio = static_cast<double>(best_secondary_inliers) / outlier_indices.size();
                ROS_DEBUG("Secondary plane inlier ratio in outliers: %.1f%% (min: 30%%)", secondary_ratio * 100.0);
                
                if (secondary_ratio >= 0.3) {  // 至少30%的外点属于第二平面
                    ROS_DEBUG("Found valid secondary plane: %d inliers, angle=%.1f deg, ratio=%.1f%%", 
                             best_secondary_inliers, angle, secondary_ratio * 100.0);
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /**
     * @brief 计算综合质量评分
     */
    void calculate_overall_score() {
        double score = 0.0;
        
        // 内点比例权重 (40%)
        score += 0.4 * std::min(1.0, plane_quality_.inlier_ratio / min_inlier_ratio_);
        
        // 残差质量权重 (30%)
        double residual_score = 1.0 - std::min(1.0, plane_quality_.average_residual / max_average_residual_);
        score += 0.3 * residual_score;
        
        // 轴对齐度权重 (20%)
        score += 0.2 * plane_quality_.normal_axis_alignment;
        
        // 单一平面权重 (10%)
        score += 0.1 * (plane_quality_.is_single_plane ? 1.0 : 0.0);
        
        plane_quality_.overall_score = std::max(0.0, std::min(1.0, score));
    }

    /**
     * @brief 随机采样指定数量的点索引
     */
    std::vector<int> random_sample(int count) {
        if (count > static_cast<int>(points_.size())) {
            return {};
        }
        
        std::vector<int> indices;
        std::vector<int> available_indices(points_.size());
        std::iota(available_indices.begin(), available_indices.end(), 0);
        
        for (int i = 0; i < count; ++i) {
            if (available_indices.empty()) break;
            
            int random_idx = static_cast<int>(dis_(gen_) * available_indices.size());
            indices.push_back(available_indices[random_idx]);
            available_indices.erase(available_indices.begin() + random_idx);
        }
        
        return indices;
    }
    
    /**
     * @brief 检查三点是否共线
     */
    bool are_collinear(const Point3D& a, const Point3D& b, const Point3D& c) const {
        Point3D ab = b - a;
        Point3D ac = c - a;
        Point3D cross_product = ab.cross(ac);
        return cross_product.norm() < 1e-6;
    }
    
    /**
     * @brief 根据三个点拟合平面
     */
    bool fit_plane_from_three_points(const Point3D& a, const Point3D& b, const Point3D& c, Plane& plane) {
        Point3D ab = b - a;
        Point3D ac = c - a;
        Point3D normal = ab.cross(ac);
        
        if (normal.norm() < 1e-8) {
            return false; // 三点共线
        }
        
        normal.normalize();
        plane = Plane(a, normal);
        return true;
    }
};

#endif // COLLISION_RANSAC_HPP 