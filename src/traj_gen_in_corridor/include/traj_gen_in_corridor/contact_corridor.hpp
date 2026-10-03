#ifndef CONTACT_CORRIDOR_HPP
#define CONTACT_CORRIDOR_HPP
#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <limits>

namespace contact_corridor {
using Poly = Eigen::Matrix<double, 6, -1>;
using Polys = std::vector<Poly>;
inline double violation(const Poly &poly, const Eigen::Vector3d &p) {
    double result = 0.0;
    if (!p.allFinite() || poly.cols() == 0) return INFINITY;
    for (int j = 0; j < poly.cols(); ++j) {
        const Eigen::Vector3d n = poly.col(j).head<3>();
        const double len = n.norm();
        if (!poly.col(j).allFinite() || len < 1e-9) return INFINITY;
        result = std::max(result, n.dot(p - poly.col(j).tail<3>()) / len);
    }
    return result;
}

// 原走廊保持不变。只在指定碰撞点附近建立有界接触区：
// 允许与碰撞墙同向的障碍面恢复膨胀余量，地板、天花板和其他朝向的面保持原约束。
// 不允许延伸超过接触点所在平面，也不允许把这一例外用于整条走廊。
inline bool makePatch(const Polys &raw, const Eigen::Vector3d &point,
                      const Eigen::Vector3d &inward, double extension_limit,
                      double tangent_radius, double approach_depth,
                      Poly &patch, int &parent, double &extension,
                      int preferred_parent = -1) {
    parent = -1;
    extension = INFINITY;
    if (!point.allFinite() || !inward.allFinite() || inward.norm() < 1e-6 ||
        !std::isfinite(extension_limit) || extension_limit < 0.0 ||
        !std::isfinite(tangent_radius) || tangent_radius <= 0.0 ||
        !std::isfinite(approach_depth) || approach_depth <= 0.0) return false;
    const Eigen::Vector3d outward = -inward.normalized();
    Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(outward);
    if (tangent.norm() < 1e-6) return false; // 本实验只处理墙面接触
    tangent.normalize();
    const Eigen::Vector3d tangent2 = outward.cross(tangent).normalized();
    for (size_t i = 0; i < raw.size(); ++i) {
        Poly candidate = raw[i];
        double needed = 0.0;
        bool valid = candidate.cols() > 0;
        for (int j = 0; valid && j < candidate.cols(); ++j) {
            Eigen::Vector3d n = candidate.col(j).head<3>();
            if (!candidate.col(j).allFinite() || n.norm() < 1e-9) {valid = false; break;}
            n.normalize();
            candidate.col(j).head<3>() = n;
            const double distance = n.dot(point - candidate.col(j).tail<3>());
            if (distance <= 1e-6) continue;
            // 只允许局部恢复同向墙面的膨胀余量，不移动地板/天花板。
            if (n.dot(outward) < 0.5 || std::abs(n.z()) > 0.95 || distance > extension_limit) {
                valid = false; break;
            }
            candidate.col(j).tail<3>() += distance * n;
            needed = std::max(needed, distance);
        }
        if (!valid) continue;
        if (parent >= 0 &&
            (preferred_parent < 0 ? needed >= extension :
             (std::abs(static_cast<int>(i) - preferred_parent) > std::abs(parent - preferred_parent) ||
              (std::abs(static_cast<int>(i) - preferred_parent) == std::abs(parent - preferred_parent) &&
               needed >= extension)))) continue;
        const int cols = candidate.cols();
        candidate.conservativeResize(6, cols + 6);
        const Eigen::Vector3d normals[6] = {outward, -outward, tangent, -tangent, tangent2, -tangent2};
        const double offsets[6] = {0.0, approach_depth, tangent_radius, tangent_radius, tangent_radius, tangent_radius};
        for (int j = 0; j < 6; ++j) {
            candidate.col(cols + j).head<3>() = normals[j];
            candidate.col(cols + j).tail<3>() = point + offsets[j] * normals[j];
        }
        patch = candidate;
        parent = static_cast<int>(i);
        extension = needed;
    }
    return parent >= 0;
}

inline bool buildSegment(const Polys &raw, const Poly &patch, int parent,
                         const Eigen::Vector3d &other_endpoint, bool to_contact, Polys &out) {
    int other = -1;
    for (int i = 0; i < static_cast<int>(raw.size()); ++i) {
        if (violation(raw[i], other_endpoint) <= 1e-6 &&
            (other < 0 || std::abs(i - parent) < std::abs(other - parent))) other = i;
    }
    if (other < 0 || parent < 0 || parent >= static_cast<int>(raw.size())) return false;
    out.clear();
    int from = to_contact ? other : parent;
    int to = to_contact ? parent : other;
    if (!to_contact) out.push_back(patch);
    const int step = from <= to ? 1 : -1;
    for (int i = from; ; i += step) {out.push_back(raw[i]); if (i == to) break;}
    if (to_contact) out.push_back(patch);
    return true;
}
}
#endif
