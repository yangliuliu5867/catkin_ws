#ifndef _SAMPLE_FORWARD_HPP_
#define _SAMPLE_FORWARD_HPP_

#include <vector>
#include <list>
#include <iostream>
#include <memory>
#include <array>
#include <random>
#include <cmath>
#include <complex>
#include <functional>
#include <set>
#include <limits>
#include <algorithm>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <unistd.h>
#include <iomanip>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/TrajectoryPlan.h>
#include <quadrotor_msgs/CollisionTrajectory.h>
#include <quadrotor_msgs/CollisionEvent.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include "config.hpp"
#include "collision_ransac.hpp"
#include "intention_get_corridor/glbmap.hpp"
#include "trt_manager.hpp"

// Utility: resolve a relative data_ path to workspace-root/data_/filename
static inline std::string ensureTrailingSlashLocal(std::string path)
{
	if (path.empty())
		return path;
	if (path.back() != '/')
		path.push_back('/');
	return path;
}

static inline std::string resolveDataCsvPathIntention(const std::string &filename, const std::string &data_dir = "data_/")
{
	std::string dir = data_dir.empty() ? std::string("data_/") : data_dir;
	// absolute path already
	if (!dir.empty() && dir.front() == '/') {
		dir = ensureTrailingSlashLocal(dir);
		return dir + filename;
	}
	// attempt to find workspace root via this package path
	std::string pkg = ros::package::getPath("intention_get_corridor");
	const std::string needle = "/src/";
	std::string root = pkg;
	auto pos = pkg.rfind(needle);
	if (pos != std::string::npos)
		root = pkg.substr(0, pos);
	if (!root.empty() && root.back() != '/')
		root.push_back('/');
	dir = ensureTrailingSlashLocal(dir);
	return root + dir + filename;
}


// Default B-spline parameters used by sample forward helpers
constexpr int SAMPLE_FORWARD_BSPLINE_DEGREE = 5;
// NOTE: Student model remap now returns 18 control points for avoidance trajectories.
// Update control-point count to match remapped avoidance outputs.
constexpr int SAMPLE_FORWARD_BSPLINE_CTRL_N = 18;
// All Student-generated candidate trajectories are fixed-duration.
constexpr double SAMPLE_FORWARD_FIXED_DURATION_S = 4.0;

// ===== Forward-sampling generators (templates) =====
namespace sample_forward {

// Forward declaration for use inside templates defined later in this namespace.
inline quadrotor_msgs::TrajectoryPlan makeTwoPointMinJerkPlan(
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &dir_unit,
	int mode);

inline quadrotor_msgs::TrajectoryPlan buildStraightLineBsplineCtrlPlan(
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &dir_unit,
	int mode);

inline Eigen::Vector2d evalClampedUniformBspline2D(
	const std::array<Eigen::Vector2d, SAMPLE_FORWARD_BSPLINE_CTRL_N> &ctrl,
	double u);
inline Eigen::Vector2d evalClampedUniformBspline2DVar(
	const std::vector<Eigen::Vector2d> &ctrl,
	int degree,
	double u);
// Templated overload declaration so calls with different control-point counts
// (e.g. 16 for collision pre, 18 for avoidance) resolve correctly.
template <size_t N>
inline Eigen::Vector2d evalClampedUniformBspline2D(
    const std::array<Eigen::Vector2d, N> &ctrl,
    double u);
// forward declarations for functions defined later but used above
inline void resampleMethod2ArcLength(
	const std::vector<Eigen::Vector3d> &dense_pts,
	const std::vector<double> &dense_t,
	double spacing_m,
	std::vector<Eigen::Vector3d> &out_pts,
	std::vector<double> &out_t);

inline quadrotor_msgs::TrajectoryPlan buildWaypointsPlanFromSamples(
	int mode,
	const std::vector<Eigen::Vector3d> &pts,
	const std::vector<double> &ts,
	const Eigen::Vector3d &dir_unit);

template <typename PlaneCandidatesVec>
inline bool straightLineRmsViolationExceeds(
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const PlaneCandidatesVec &plane_candidates,
	double &out_rms,
	double &out_max,
	const double rms_tol = 5e-3,
	const double hard_tol = 0.20)
{
	out_rms = 0.0;
	out_max = 0.0;

	const Eigen::Vector3d d = (p1 - p0);
	const double dist = d.norm();
	if (!std::isfinite(dist) || dist < 1e-6) {
		return false;
	}

	// group candidate indices by corridor_idx (polytope id)
	std::unordered_map<int, std::vector<size_t>> groups;
	groups.reserve(16);
	for (size_t i = 0; i < plane_candidates.size(); ++i) {
		const auto &c = plane_candidates[i];
		groups[c.corridor_idx].push_back(i);
	}
	if (groups.empty()) {
		return false;
	}

	const int samples = std::max(20, static_cast<int>(std::ceil(dist * 10.0)));
	double accum_sq = 0.0;
	int cnt = 0;
	for (int si = 0; si <= samples; ++si) {
		const double a = static_cast<double>(si) / static_cast<double>(samples);
		const Eigen::Vector3d p = p0 + a * d;

		double best_poly_violation = std::numeric_limits<double>::infinity();
		for (const auto &kv : groups) {
			const auto &idxs = kv.second;
			double poly_violation = 0.0;
			for (const size_t idx : idxs) {
				const auto &cand = plane_candidates[idx];
				const Eigen::Vector3d &n = cand.normal_unit;
				if (!n.allFinite()) continue;
				const double v = n.dot(p) + cand.d;
				if (v > poly_violation) poly_violation = v;
			}
			if (poly_violation < best_poly_violation) best_poly_violation = poly_violation;
		}

		if (!std::isfinite(best_poly_violation)) continue;
		if (best_poly_violation > out_max) out_max = best_poly_violation;
		if (best_poly_violation > 0.0) {
			accum_sq += best_poly_violation * best_poly_violation;
			cnt += 1;
		}
	}

	if (cnt > 0) {
		out_rms = std::sqrt(accum_sq / static_cast<double>(cnt));
	}

	return (out_max > hard_tol) || (out_rms > rms_tol);
}

inline quadrotor_msgs::TrajectoryPlan makeEmptyPlan(int mode)
{
	quadrotor_msgs::TrajectoryPlan out;
	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	out.trajectory_mode = mode;
	// Default empty plan: waypoint representation
	out.representation = quadrotor_msgs::TrajectoryPlan::REP_WAYPOINTS;
	out.bspline_degree = 0;
	out.waypoints.clear();
	out.headings.clear();
	out.position_constraints.clear();
	out.segment_times.clear();
	return out;
}

inline double yawFromQuat(const geometry_msgs::Quaternion &q)
{
	// Standard yaw (Z) from quaternion.
	// yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
	const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

inline Eigen::Matrix2d rotWorldToBody2DFromForward(const Eigen::Vector2d &forward_world_unit)
{
	Eigen::Vector2d f = forward_world_unit;
	if (!f.allFinite() || f.norm() < 1e-6)
	{
		f = Eigen::Vector2d(1.0, 0.0);
	}
	else
	{
		f.normalize();
	}
	const double theta = std::atan2(f.y(), f.x());
	const double c = std::cos(theta);
	const double s = std::sin(theta);
	// local = R(-theta) * world
	Eigen::Matrix2d R_wb;
	R_wb << c, s,
		   -s, c;
	return R_wb;
}

inline void fillBsplinePlanFromWorldCtrlPoints(
	quadrotor_msgs::TrajectoryPlan &out,
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &preferred_dir_unit,
	const std::array<Eigen::Vector2d, SAMPLE_FORWARD_BSPLINE_CTRL_N> &ctrl_xy_world,
	int mode)
{
	constexpr int kDegree = SAMPLE_FORWARD_BSPLINE_DEGREE;
	constexpr int kCtrlN = SAMPLE_FORWARD_BSPLINE_CTRL_N;
	constexpr int kSpans = kCtrlN - kDegree;
	static_assert(kSpans > 0, "Invalid B-spline spans");

	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	out.trajectory_mode = mode;

	// IMPORTANT: internal candidate plans use BSPLINE control points for scoring/selection.
	// We only convert the *final* selected plan to REP_WAYPOINTS right before publishing.
	out.representation = quadrotor_msgs::TrajectoryPlan::REP_BSPLINE;
	out.bspline_degree = kDegree;

	out.waypoints.resize(kCtrlN);
	out.headings.resize(kCtrlN);
	out.position_constraints.resize(kCtrlN);
	out.segment_times.clear();
	out.segment_times.reserve(kSpans);

	for (int i = 0; i < kCtrlN; ++i)
	{
		geometry_msgs::Point pt;
		pt.x = ctrl_xy_world[i].x();
		pt.y = ctrl_xy_world[i].y();
		// Z: interpolate between p0.z and p1.z using a min-jerk quintic profile
		double a = static_cast<double>(i) / static_cast<double>(kCtrlN - 1);
		double a2 = a * a;
		double a3 = a2 * a;
		double a4 = a3 * a;
		double a5 = a4 * a;
		double h = 10.0 * a3 - 15.0 * a4 + 6.0 * a5;
		pt.z = p0.z() + (p1.z() - p0.z()) * h;
		out.waypoints[i] = pt;
		out.position_constraints[i] = false;
	}

	Eigen::Vector3d dir = preferred_dir_unit;
	dir.z() = 0.0;
	if (dir.head<2>().norm() > 1e-6)
		dir.normalize();
	else
	{
		Eigen::Vector3d d = (p1 - p0);
		d.z() = 0.0;
		if (d.head<2>().norm() > 1e-6)
			dir = d.normalized();
		else
			dir = Eigen::Vector3d(1.0, 0.0, 0.0);
	}
	geometry_msgs::Vector3 h;
	h.x = dir.x();
	h.y = dir.y();
	h.z = 0.0;
	for (int i = 0; i < kCtrlN; ++i)
		out.headings[i] = h;

	// Uniform knot-span times for fixed-duration student trajectories.
	const double dt = SAMPLE_FORWARD_FIXED_DURATION_S / static_cast<double>(kSpans);
	for (int i = 0; i < kSpans; ++i)
		out.segment_times.push_back(dt);
}

inline void fillBsplinePlanFromWorldCtrlPointsVar(
	quadrotor_msgs::TrajectoryPlan &out,
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &preferred_dir_unit,
	const std::vector<Eigen::Vector2d> &ctrl_xy_world,
	int degree,
	int mode)
{
	const int kCtrlN = static_cast<int>(ctrl_xy_world.size());
	const int kDegree = degree;
	const int kSpans = kCtrlN - kDegree;
	if (kCtrlN <= 0 || kSpans <= 0)
	{
		out = makeEmptyPlan(mode);
		return;
	}

	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	out.trajectory_mode = mode;
	out.representation = quadrotor_msgs::TrajectoryPlan::REP_BSPLINE;
	out.bspline_degree = kDegree;

	out.waypoints.resize(kCtrlN);
	out.headings.resize(kCtrlN);
	out.position_constraints.resize(kCtrlN);
	out.segment_times.clear();
	out.segment_times.reserve(kSpans);

	for (int i = 0; i < kCtrlN; ++i)
	{
		geometry_msgs::Point pt;
		pt.x = ctrl_xy_world[i].x();
		pt.y = ctrl_xy_world[i].y();
		// Z: interpolate between p0.z and p1.z using a min-jerk quintic profile
		double a = static_cast<double>(i) / static_cast<double>(kCtrlN - 1);
		double a2 = a * a;
		double a3 = a2 * a;
		double a4 = a3 * a;
		double a5 = a4 * a;
		double h = 10.0 * a3 - 15.0 * a4 + 6.0 * a5;
		pt.z = p0.z() + (p1.z() - p0.z()) * h;
		out.waypoints[i] = pt;
		out.position_constraints[i] = false;
	}

	Eigen::Vector3d dir = preferred_dir_unit;
	dir.z() = 0.0;
	if (dir.head<2>().norm() > 1e-6)
		dir.normalize();
	else
	{
		Eigen::Vector3d d = (p1 - p0);
		d.z() = 0.0;
		if (d.head<2>().norm() > 1e-6)
			d.normalize();
		else
			d = Eigen::Vector3d(1.0, 0.0, 0.0);
		dir = d;
	}
	geometry_msgs::Vector3 h;
	h.x = dir.x();
	h.y = dir.y();
	h.z = 0.0;
	for (int i = 0; i < kCtrlN; ++i)
		out.headings[i] = h;

	const double dt = SAMPLE_FORWARD_FIXED_DURATION_S / static_cast<double>(kSpans);
	for (int i = 0; i < kSpans; ++i)
		out.segment_times.push_back(dt);
}

inline quadrotor_msgs::TrajectoryPlan convertBsplineCtrlPlanToWaypointsMethod2Var(
	const quadrotor_msgs::TrajectoryPlan &plan_ctrl,
	const Eigen::Vector3d &dir_unit,
	double duration_s,
	double u_ratio = 1.0)
{
	const int mode = plan_ctrl.trajectory_mode;
	const int degree = plan_ctrl.bspline_degree;
	const int n = static_cast<int>(plan_ctrl.waypoints.size());
	const int spans = n - degree;
	if (n <= 0 || spans <= 0)
		return makeEmptyPlan(mode);
	const double u_end = std::clamp(u_ratio, 0.0, 1.0) * static_cast<double>(spans);

	std::vector<Eigen::Vector2d> ctrl;
	ctrl.reserve(static_cast<size_t>(n));
	for (int j = 0; j < n; ++j)
	{
		const auto &pt = plan_ctrl.waypoints[j];
		ctrl.emplace_back(pt.x, pt.y);
	}
	const double z0 = plan_ctrl.waypoints.front().z;
	const double z1 = plan_ctrl.waypoints.back().z;

	const int M = std::max(60, spans * 20 + 1);
	std::vector<Eigen::Vector3d> dense_pts;
	std::vector<double> dense_t;
	dense_pts.reserve(M);
	dense_t.reserve(M);

	for (int k = 0; k < M; ++k)
	{
		const double u = (static_cast<double>(k) / static_cast<double>(M - 1)) * u_end;
		const Eigen::Vector2d xy = evalClampedUniformBspline2DVar(ctrl, degree, u);
		double s_norm = (u_end > 1e-12) ? (u / u_end) : 0.0;
		if (s_norm < 0.0) s_norm = 0.0;
		if (s_norm > 1.0) s_norm = 1.0;
		double s2 = s_norm * s_norm;
		double s3 = s2 * s_norm;
		double s4 = s3 * s_norm;
		double s5 = s4 * s_norm;
		double h = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
		dense_pts.emplace_back(xy.x(), xy.y(), z0 + (z1 - z0) * h);
		const double t = (static_cast<double>(k) / static_cast<double>(M - 1)) * duration_s;
		dense_t.push_back(t);
	}

	std::vector<Eigen::Vector3d> pts;
	std::vector<double> ts;
	resampleMethod2ArcLength(dense_pts, dense_t, 1.0, pts, ts);
	if (pts.size() < 2)
	{
		const Eigen::Vector2d end_xy = evalClampedUniformBspline2DVar(ctrl, degree, u_end);
		pts.clear();
		ts.clear();
		pts.push_back(Eigen::Vector3d(plan_ctrl.waypoints.front().x, plan_ctrl.waypoints.front().y, z0));
		ts.push_back(0.0);
		pts.push_back(Eigen::Vector3d(end_xy.x(), end_xy.y(), z1));
		ts.push_back(duration_s);
	}

	auto out = buildWaypointsPlanFromSamples(mode, pts, ts, dir_unit);
	out.header = plan_ctrl.header;
	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	return out;
}
inline void readMaxVelAccFromRosParams(double &out_max_vel, double &out_max_acc)
{
	static double cached_vel = std::numeric_limits<double>::quiet_NaN();
	static double cached_acc = std::numeric_limits<double>::quiet_NaN();
	if (std::isfinite(cached_vel) && std::isfinite(cached_acc))
	{
		out_max_vel = cached_vel;
		out_max_acc = cached_acc;
		return;
	}

	ros::NodeHandle pnh("~");
	pnh.param("MaxVelocity", cached_vel, 4.0);
	pnh.param("MaxAcceleration", cached_acc, 4.0);
	if (!std::isfinite(cached_vel) || cached_vel <= 1e-6) cached_vel = 4.0;
	if (!std::isfinite(cached_acc) || cached_acc <= 1e-6) cached_acc = 4.0;
	out_max_vel = cached_vel;
	out_max_acc = cached_acc;
}

inline double clamp01(double x)
{
	if (x < 0.0) return 0.0;
	if (x > 1.0) return 1.0;
	return x;
}

inline double clamp(double x, double lo, double hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

inline Eigen::Vector2d evalClampedUniformBspline2DVar(
	const std::vector<Eigen::Vector2d> &ctrl,
	int degree,
	double u)
{
	const int n = static_cast<int>(ctrl.size());
	const int p = degree;
	const int spans = n - p;
	if (n <= 0)
		return Eigen::Vector2d::Zero();
	if (n <= p)
		return ctrl.back();
	if (!std::isfinite(u)) u = 0.0;
	if (u <= 0.0) return ctrl.front();
	if (u >= static_cast<double>(spans)) return ctrl.back();

	int k = p + static_cast<int>(std::floor(u));
	if (k < p) k = p;
	if (k > n - 1) k = n - 1;

	std::vector<Eigen::Vector2d> d(static_cast<size_t>(p + 1), Eigen::Vector2d::Zero());
	for (int j = 0; j <= p; ++j)
		d[static_cast<size_t>(j)] = ctrl[k - p + j];

	auto knot = [&](int i) -> double {
		if (i <= p) return 0.0;
		if (i >= spans) return static_cast<double>(spans);
		return static_cast<double>(i - p);
	};

	for (int r = 1; r <= p; ++r)
	{
		for (int j = p; j >= r; --j)
		{
			const int idx = k - p + j;
			const double denom = knot(idx + p + 1 - r) - knot(idx);
			double alpha = 0.0;
			if (std::abs(denom) > 1e-12)
				alpha = (u - knot(idx)) / denom;
			alpha = clamp(alpha, 0.0, 1.0);
			d[static_cast<size_t>(j)] = (1.0 - alpha) * d[static_cast<size_t>(j - 1)] + alpha * d[static_cast<size_t>(j)];
		}
	}

	return d[static_cast<size_t>(p)];
}

inline void resampleMethod2ArcLength(
	const std::vector<Eigen::Vector3d> &dense_pts,
	const std::vector<double> &dense_t,
	double spacing_m,
	std::vector<Eigen::Vector3d> &out_pts,
	std::vector<double> &out_t)
{
	out_pts.clear();
	out_t.clear();
	if (dense_pts.size() < 2 || dense_pts.size() != dense_t.size())
		return;

	const size_t M = dense_pts.size();
	std::vector<double> s(M, 0.0);
	for (size_t i = 1; i < M; ++i)
	{
		s[i] = s[i - 1] + (dense_pts[i] - dense_pts[i - 1]).norm();
	}
	const double total_len = s.back();

	// Always keep endpoints
	out_pts.push_back(dense_pts.front());
	out_t.push_back(dense_t.front());

	if (!std::isfinite(total_len) || total_len < 1e-9)
	{
		out_pts.push_back(dense_pts.back());
		out_t.push_back(dense_t.back());
		return;
	}

	if (!std::isfinite(spacing_m) || spacing_m <= 1e-6)
		spacing_m = 1.0;

	// target arc-lengths: 0, spacing, 2*spacing, ..., total
	double next_s = spacing_m;
	size_t i = 1;
	while (next_s < total_len - 1e-6)
	{
		while (i < M && s[i] < next_s)
			++i;
		if (i >= M)
			break;
		const size_t i0 = i - 1;
		const size_t i1 = i;
		const double s0 = s[i0];
		const double s1 = s[i1];
		double a = 0.0;
		if (s1 > s0 + 1e-12)
			a = (next_s - s0) / (s1 - s0);
		a = clamp(a, 0.0, 1.0);
		const Eigen::Vector3d p = dense_pts[i0] + a * (dense_pts[i1] - dense_pts[i0]);
		const double t = dense_t[i0] + a * (dense_t[i1] - dense_t[i0]);
		out_pts.push_back(p);
		out_t.push_back(t);
		next_s += spacing_m;
	}

	out_pts.push_back(dense_pts.back());
	out_t.push_back(dense_t.back());

	// merge final short segment (<0.5m): remove the penultimate point
	if (out_pts.size() >= 3)
	{
		const size_t N = out_pts.size();
		const double last_len = (out_pts[N - 1] - out_pts[N - 2]).norm();
		if (std::isfinite(last_len) && last_len < 0.5)
		{
			out_pts.erase(out_pts.begin() + static_cast<long>(N - 2));
			out_t.erase(out_t.begin() + static_cast<long>(N - 2));
		}
	}
}

inline quadrotor_msgs::TrajectoryPlan buildWaypointsPlanFromSamples(
	int mode,
	const std::vector<Eigen::Vector3d> &pts,
	const std::vector<double> &ts,
	const Eigen::Vector3d &dir_unit)
{
	quadrotor_msgs::TrajectoryPlan out = makeEmptyPlan(mode);
	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	out.trajectory_mode = mode;
	out.representation = quadrotor_msgs::TrajectoryPlan::REP_WAYPOINTS;
	out.bspline_degree = 0;

	if (pts.size() < 2 || pts.size() != ts.size())
		return out;

	const size_t N = pts.size();
	out.waypoints.resize(N);
	out.headings.resize(N);
	out.position_constraints.resize(N);
	out.segment_times.clear();
	out.segment_times.reserve(N - 1);

	Eigen::Vector3d dir = dir_unit;
	dir.z() = 0.0;
	if (dir.head<2>().norm() > 1e-6)
		dir.normalize();
	else
		dir = Eigen::Vector3d(1.0, 0.0, 0.0);
	geometry_msgs::Vector3 h;
	h.x = dir.x();
	h.y = dir.y();
	h.z = 0.0;

	for (size_t i = 0; i < N; ++i)
	{
		geometry_msgs::Point p;
		p.x = pts[i].x();
		p.y = pts[i].y();
		p.z = pts[i].z();
		out.waypoints[i] = p;
		out.headings[i] = h;
		out.position_constraints[i] = false;
	}

	for (size_t i = 0; i + 1 < N; ++i)
	{
		double dt = ts[i + 1] - ts[i];
		if (!std::isfinite(dt) || dt <= 1e-6)
			dt = 1e-3;
		out.segment_times.push_back(dt);
	}

	return out;
}

struct Quintic1D
{
	double a0 = 0.0, a1 = 0.0, a2 = 0.0, a3 = 0.0, a4 = 0.0, a5 = 0.0;
};

inline Quintic1D fitQuinticBoundary1D(double p0, double v0, double acc0, double p1, double v1, double acc1, double T)
{
	Quintic1D q;
	if (!std::isfinite(T) || T <= 1e-6)
	{
		q.a0 = p0;
		q.a1 = 0.0;
		q.a2 = 0.0;
		q.a3 = 0.0;
		q.a4 = 0.0;
		q.a5 = 0.0;
		return q;
	}

	q.a0 = p0;
	q.a1 = v0;
	q.a2 = 0.5 * acc0;

	const double T2 = T * T;
	const double T3 = T2 * T;
	const double T4 = T3 * T;
	const double T5 = T4 * T;

	const double c0 = p1 - (q.a0 + q.a1 * T + q.a2 * T2);
	const double c1 = v1 - (q.a1 + 2.0 * q.a2 * T);
	const double c2 = acc1 - (2.0 * q.a2);

	Eigen::Matrix3d A;
	A << T3, T4, T5,
		 3.0 * T2, 4.0 * T3, 5.0 * T4,
		 6.0 * T, 12.0 * T2, 20.0 * T3;
	Eigen::Vector3d b(c0, c1, c2);
	Eigen::Vector3d x = A.fullPivLu().solve(b);
	q.a3 = x(0);
	q.a4 = x(1);
	q.a5 = x(2);
	return q;
}

inline double evalQuintic1D(const Quintic1D &q, double t)
{
	if (!std::isfinite(t)) t = 0.0;
	const double t2 = t * t;
	const double t3 = t2 * t;
	const double t4 = t3 * t;
	const double t5 = t4 * t;
	return q.a0 + q.a1 * t + q.a2 * t2 + q.a3 * t3 + q.a4 * t4 + q.a5 * t5;
}

inline void buildPostCollisionMinJerkDenseSamples(
	const Eigen::Vector3d &p_coll,
	const Eigen::Vector3d &v_post_world,
	double T_post,
	std::vector<Eigen::Vector3d> &dense_pts,
	std::vector<double> &dense_t)
{
	dense_pts.clear();
	dense_t.clear();
	if (!std::isfinite(T_post) || T_post <= 1e-6)
	{
		dense_pts.push_back(p_coll);
		dense_t.push_back(0.0);
		dense_pts.push_back(p_coll);
		dense_t.push_back(std::max(0.0, T_post));
		return;
	}

	// Use the same displacement model as existing reconstruction: average speed = v/2 over T => dp = 0.5*v*T.
	Eigen::Vector3d v0 = v_post_world;
	v0.z() = 0.0;
	if (!v0.allFinite()) v0 = Eigen::Vector3d::Zero();
	const Eigen::Vector3d p_end = p_coll + v0 * T_post;  // *0.5 removed for consistency with existing reconstruction

	const Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
	const Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
	const Eigen::Vector3d a1 = Eigen::Vector3d::Zero();

	const Quintic1D qx = fitQuinticBoundary1D(p_coll.x(), v0.x(), a0.x(), p_end.x(), v1.x(), a1.x(), T_post);
	const Quintic1D qy = fitQuinticBoundary1D(p_coll.y(), v0.y(), a0.y(), p_end.y(), v1.y(), a1.y(), T_post);

	const int M = 220;
	dense_pts.reserve(M);
	dense_t.reserve(M);
	for (int k = 0; k < M; ++k)
	{
		const double t = (static_cast<double>(k) / static_cast<double>(M - 1)) * T_post;
		Eigen::Vector3d p;
		p.x() = evalQuintic1D(qx, t);
		p.y() = evalQuintic1D(qy, t);
		p.z() = p_coll.z();
		dense_pts.push_back(p);
		dense_t.push_back(t);
	}
}

inline quadrotor_msgs::TrajectoryPlan convertClampedUniformBsplineCtrlPlanToWaypointsMethod2(
	const quadrotor_msgs::TrajectoryPlan &plan_ctrl,
	const Eigen::Vector3d &dir_unit,
	double duration_s)
{
	const int mode = plan_ctrl.trajectory_mode;
	if (plan_ctrl.waypoints.size() < SAMPLE_FORWARD_BSPLINE_CTRL_N)
		return makeEmptyPlan(mode);

	std::array<Eigen::Vector2d, SAMPLE_FORWARD_BSPLINE_CTRL_N> ctrl;
	for (int j = 0; j < SAMPLE_FORWARD_BSPLINE_CTRL_N; ++j)
	{
		const auto &pt = plan_ctrl.waypoints[j];
		ctrl[j] = Eigen::Vector2d(pt.x, pt.y);
	}
	const double z0 = plan_ctrl.waypoints.front().z;
	const double z1 = plan_ctrl.waypoints.back().z;

	const int spans = SAMPLE_FORWARD_BSPLINE_CTRL_N - SAMPLE_FORWARD_BSPLINE_DEGREE;
	const int M = std::max(60, spans * 20 + 1);
	std::vector<Eigen::Vector3d> dense_pts;
	std::vector<double> dense_t;
	dense_pts.reserve(M);
	dense_t.reserve(M);

	for (int k = 0; k < M; ++k)
	{
		const double u = (static_cast<double>(k) / static_cast<double>(M - 1)) * static_cast<double>(spans);
		const Eigen::Vector2d xy = evalClampedUniformBspline2D(ctrl, u);
		double s_norm = (static_cast<double>(k) / static_cast<double>(M - 1));
		// u ranges 0..spans; normalize by spans
		double spans_d = static_cast<double>(spans);
		double s = (spans_d > 1e-12) ? ( ( (static_cast<double>(k) / static_cast<double>(M - 1)) * spans_d ) / spans_d ) : s_norm;
		if (s < 0.0) s = 0.0;
		if (s > 1.0) s = 1.0;
		double s2 = s * s;
		double s3 = s2 * s;
		double s4 = s3 * s;
		double s5 = s4 * s;
		double h = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
		dense_pts.emplace_back(xy.x(), xy.y(), z0 + (z1 - z0) * h);
		const double t = (static_cast<double>(k) / static_cast<double>(M - 1)) * duration_s;
		dense_t.push_back(t);
	}

	std::vector<Eigen::Vector3d> pts;
	std::vector<double> ts;
	resampleMethod2ArcLength(dense_pts, dense_t, 1.0, pts, ts);
	if (pts.size() < 2)
	{
			pts.clear();
			ts.clear();
			pts.push_back(Eigen::Vector3d(plan_ctrl.waypoints.front().x, plan_ctrl.waypoints.front().y, z0));
			ts.push_back(0.0);
			pts.push_back(Eigen::Vector3d(plan_ctrl.waypoints.back().x, plan_ctrl.waypoints.back().y, z1));
			ts.push_back(duration_s);
	}

	auto out = buildWaypointsPlanFromSamples(mode, pts, ts, dir_unit);
	out.header = plan_ctrl.header;
	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	return out;
}

inline quadrotor_msgs::TrajectoryPlan convertCollisionPreBsplineCtrlPlanToWaypointsMethod2(
	const quadrotor_msgs::TrajectoryPlan &plan_ctrl,
	const Eigen::Vector3d &dir_unit,
	double pre_ratio)
{
	const int mode = plan_ctrl.trajectory_mode;
	if (plan_ctrl.waypoints.size() < SAMPLE_FORWARD_BSPLINE_CTRL_N)
		return makeEmptyPlan(mode);

	// Collision pre-segment control points: P0..P11 (12 points), degree=5 => spans=7
	constexpr int kDegree = SAMPLE_FORWARD_BSPLINE_DEGREE;
	constexpr int kPreCtrlN = 12;
	constexpr int kSpansPre = kPreCtrlN - kDegree;
	static_assert(kSpansPre > 0, "Invalid collision pre spans");

	std::vector<Eigen::Vector2d> ctrl;
	ctrl.reserve(kPreCtrlN);
	for (int j = 0; j < kPreCtrlN; ++j)
	{
		const auto &pt = plan_ctrl.waypoints[j];
		ctrl.emplace_back(pt.x, pt.y);
	}
	const double z0 = plan_ctrl.waypoints.front().z;
	const double z1 = plan_ctrl.waypoints[11].z;
	const double T_pre = clamp(pre_ratio, 0.0, 1.0) * SAMPLE_FORWARD_FIXED_DURATION_S;

	const int M = std::max(40, kSpansPre * 20 + 1);
	std::vector<Eigen::Vector3d> dense_pts;
	std::vector<double> dense_t;
	dense_pts.reserve(M);
	dense_t.reserve(M);

	for (int k = 0; k < M; ++k)
	{
		const double u = (static_cast<double>(k) / static_cast<double>(M - 1)) * static_cast<double>(kSpansPre);
		const Eigen::Vector2d xy = evalClampedUniformBspline2DVar(ctrl, kDegree, u);
		double s_norm = (static_cast<double>(k) / static_cast<double>(M - 1));
		double s = (kSpansPre > 1e-12) ? ( ( (static_cast<double>(k) / static_cast<double>(M - 1)) * static_cast<double>(kSpansPre) ) / static_cast<double>(kSpansPre) ) : s_norm;
		if (s < 0.0) s = 0.0;
		if (s > 1.0) s = 1.0;
		double s2 = s * s;
		double s3 = s2 * s;
		double s4 = s3 * s;
		double s5 = s4 * s;
		double h = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
		dense_pts.emplace_back(xy.x(), xy.y(), z0 + (z1 - z0) * h);
		const double t = (static_cast<double>(k) / static_cast<double>(M - 1)) * T_pre;
		dense_t.push_back(t);
	}

	std::vector<Eigen::Vector3d> pts;
	std::vector<double> ts;
	resampleMethod2ArcLength(dense_pts, dense_t, 1.0, pts, ts);
	if (pts.size() < 2)
	{
		pts.clear();
		ts.clear();
		pts.push_back(Eigen::Vector3d(plan_ctrl.waypoints.front().x, plan_ctrl.waypoints.front().y, z0));
		ts.push_back(0.0);
		pts.push_back(Eigen::Vector3d(plan_ctrl.waypoints[11].x, plan_ctrl.waypoints[11].y, z1));
		ts.push_back(T_pre);
	}

	auto out = buildWaypointsPlanFromSamples(mode, pts, ts, dir_unit);
	out.header = plan_ctrl.header;
	out.header.stamp = ros::Time::now();
	out.header.frame_id = "world";
	return out;
}

inline Eigen::Vector2d evalClampedUniformBspline2D(
	const std::array<Eigen::Vector2d, SAMPLE_FORWARD_BSPLINE_CTRL_N> &ctrl,
	double u)
{
	constexpr int p = SAMPLE_FORWARD_BSPLINE_DEGREE;
	constexpr int n = SAMPLE_FORWARD_BSPLINE_CTRL_N;
	constexpr int spans = n - p; // 11

	if (!std::isfinite(u)) u = 0.0;
	if (u <= 0.0) return ctrl.front();
	if (u >= static_cast<double>(spans)) return ctrl.back();

	int k = p + static_cast<int>(std::floor(u));
	if (k < p) k = p;
	if (k > n - 1) k = n - 1;

	Eigen::Vector2d d[p + 1];
	for (int j = 0; j <= p; ++j)
	{
		d[j] = ctrl[k - p + j];
	}

	// Open-uniform clamped knots: [0..0, 1..(spans-1), spans..spans]
	auto knot = [&](int i) -> double {
		if (i <= p) return 0.0;
		if (i >= spans) return static_cast<double>(spans);
		return static_cast<double>(i - p);
	};

	for (int r = 1; r <= p; ++r)
	{
		for (int j = p; j >= r; --j)
		{
			const int idx = k - p + j;
			const double denom = knot(idx + p + 1 - r) - knot(idx);
			double alpha = 0.0;
			if (std::abs(denom) > 1e-12)
				alpha = (u - knot(idx)) / denom;
			alpha = clamp(alpha, 0.0, 1.0);
			d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
		}
	}

	return d[p];
}

template <typename PlaneCandidatesVec>
inline void buildCorridorGroupMatrices(
	const PlaneCandidatesVec &plane_candidates,
	std::vector<Eigen::MatrixXd> &out_normals,
	std::vector<Eigen::VectorXd> &out_ds)
{
	std::unordered_map<int, std::vector<size_t>> groups;
	groups.reserve(16);
	for (size_t i = 0; i < plane_candidates.size(); ++i)
	{
		const auto &c = plane_candidates[i];
		groups[c.corridor_idx].push_back(i);
	}

	out_normals.clear();
	out_ds.clear();
	out_normals.reserve(groups.size());
	out_ds.reserve(groups.size());

	for (const auto &kv : groups)
	{
		const auto &idxs = kv.second;
		Eigen::MatrixXd N(static_cast<int>(idxs.size()), 3);
		Eigen::VectorXd d(static_cast<int>(idxs.size()));
		int row = 0;
		for (const size_t idx : idxs)
		{
			const auto &cand = plane_candidates[idx];
			const Eigen::Vector3d &n = cand.normal_unit;
			if (!n.allFinite())
				continue;
			N.row(row) = n.transpose().eval();
			d(row) = cand.d;
			row++;
		}
		if (row <= 0)
			continue;
		N.conservativeResize(row, 3);
		d.conservativeResize(row);
		out_normals.push_back(N);
		out_ds.push_back(d);
	}
}

inline double corridorViolationCostVectorized(
	const Eigen::Matrix3Xd &P,
	const std::vector<Eigen::MatrixXd> &group_normals,
	const std::vector<Eigen::VectorXd> &group_ds,
	double &out_max_violation,
	double &out_rms_violation,
	const double rms_tol = 5e-2,
	const double hard_tol = 0.20)
{
	const int M = static_cast<int>(P.cols());
	out_max_violation = 0.0;
	out_rms_violation = 0.0;
	if (M <= 0 || group_normals.empty())
		return 0.0;

	Eigen::VectorXd best = Eigen::VectorXd::Constant(M, std::numeric_limits<double>::infinity());

	for (size_t gi = 0; gi < group_normals.size(); ++gi)
	{
		const Eigen::MatrixXd &N = group_normals[gi];
		const Eigen::VectorXd &d = group_ds[gi];
		Eigen::MatrixXd V = N * P; // K x M
		V.colwise() += d;
		Eigen::RowVectorXd poly = V.colwise().maxCoeff();
		Eigen::VectorXd poly_v = poly.transpose().eval();
		poly_v = poly_v.cwiseMax(0.0);
		best = best.cwiseMin(poly_v);
	}

	out_max_violation = best.maxCoeff();
	out_rms_violation = std::sqrt(best.array().square().mean());
	
	// 确保分母不为0
	const double denom_hard = std::max(1e-6, hard_tol);
	const double denom_rms  = std::max(1e-6, rms_tol);

	// c1: 最大违规代价 (Hinge Loss)
	// 如果 max_violation <= hard_tol，代价为 0
	// 如果 max_violation > hard_tol，计算超出部分的归一化值
	const double c1 = (out_max_violation > hard_tol) 
	                  ? (out_max_violation - hard_tol) / denom_hard 
	                  : 0.0;

	// c2: 均方根违规代价 (Hinge Loss)
	// 如果 rms_violation <= rms_tol，代价为 0
	// 如果 rms_violation > rms_tol，计算超出部分的归一化值
	const double c2 = (out_rms_violation > rms_tol) 
	                  ? (out_rms_violation - rms_tol) / denom_rms 
	                  : 0.0;

	return c1 + c2;
}

inline Eigen::ArrayXd corridorViolationCostBatchVectorized(
	const Eigen::Matrix3Xd &P_big,
	const int M,
	const int B,
	const std::vector<Eigen::MatrixXd> &group_normals,
	const std::vector<Eigen::VectorXd> &group_ds,
	const double rms_tol = 5e-2,
	const double hard_tol = 0.20)
{
	Eigen::ArrayXd out = Eigen::ArrayXd::Zero(B);
	if (B <= 0 || M <= 0 || group_normals.empty() || P_big.cols() != M * B)
		return out;

	Eigen::VectorXd best = Eigen::VectorXd::Constant(M * B, std::numeric_limits<double>::infinity());
	for (size_t gi = 0; gi < group_normals.size(); ++gi)
	{
		const Eigen::MatrixXd &N = group_normals[gi];
		const Eigen::VectorXd &d = group_ds[gi];
		Eigen::MatrixXd V = N * P_big; // K x (M*B)
		V.colwise() += d;
		Eigen::RowVectorXd poly = V.colwise().maxCoeff();
		Eigen::VectorXd poly_v = poly.transpose().eval();
		poly_v = poly_v.cwiseMax(0.0);
		best = best.cwiseMin(poly_v);
	}

	// Map as M x B (column-major): each column is one trajectory's samples
	Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> best_mb(best.data(), M, B);
	Eigen::RowVectorXd max_v = best_mb.colwise().maxCoeff();
	Eigen::RowVectorXd rms_v = (best_mb.array().square().colwise().mean()).sqrt().matrix();

	const double denom_hard = std::max(1e-6, hard_tol);
	const double denom_rms = std::max(1e-6, rms_tol);
	for (int bi = 0; bi < B; ++bi)
	{
		const double mv = max_v(bi);
		const double rv = rms_v(bi);
		const double c1 = (mv > hard_tol) ? (mv - hard_tol) / denom_hard : 0.0;
		const double c2 = (rv > rms_tol) ? (rv - rms_tol) / denom_rms : 0.0;
		out(bi) = c1 + c2;
	}
	return out;
}

// Masked batch corridor cost. mask_mb is M x B with entries 0/1:
// - masked-out samples do not contribute to max/rms.
inline Eigen::ArrayXd corridorViolationCostBatchVectorizedMasked(
	const Eigen::Matrix3Xd &P_big,
	const int M,
	const int B,
	const std::vector<Eigen::MatrixXd> &group_normals,
	const std::vector<Eigen::VectorXd> &group_ds,
	const Eigen::Ref<const Eigen::MatrixXd> &mask_mb,
	const double rms_tol = 5e-2,
	const double hard_tol = 0.20)
{
	Eigen::ArrayXd out = Eigen::ArrayXd::Zero(B);
	if (B <= 0 || M <= 0 || group_normals.empty() || P_big.cols() != M * B ||
		mask_mb.rows() != M || mask_mb.cols() != B)
		return out;

	Eigen::VectorXd best = Eigen::VectorXd::Constant(M * B, std::numeric_limits<double>::infinity());
	for (size_t gi = 0; gi < group_normals.size(); ++gi)
	{
		const Eigen::MatrixXd &N = group_normals[gi];
		const Eigen::VectorXd &d = group_ds[gi];
		Eigen::MatrixXd V = N * P_big; // K x (M*B)
		V.colwise() += d;
		Eigen::RowVectorXd poly = V.colwise().maxCoeff();
		Eigen::VectorXd poly_v = poly.transpose().eval();
		poly_v = poly_v.cwiseMax(0.0);
		best = best.cwiseMin(poly_v);
	}

	Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> best_m(best.data(), M, B);
	Eigen::ArrayXXd best_masked = best_m.array() * mask_mb.array();
	Eigen::RowVectorXd max_v = best_masked.colwise().maxCoeff();

	Eigen::RowVectorXd cnt = mask_mb.colwise().sum();
	for (int i = 0; i < B; ++i)
		cnt(i) = std::max(1.0, cnt(i));
	Eigen::RowVectorXd rms_v = (best_masked.square().colwise().sum().array() / cnt.array()).sqrt().matrix();

	const double denom_hard = std::max(1e-6, hard_tol);
	const double denom_rms = std::max(1e-6, rms_tol);
	for (int bi = 0; bi < B; ++bi)
	{
		const double mv = max_v(bi);
		const double rv = rms_v(bi);
		const double c1 = (mv > hard_tol) ? (mv - hard_tol) / denom_hard : 0.0;
		const double c2 = (rv > rms_tol) ? (rv - rms_tol) / denom_rms : 0.0;
		out(bi) = c1 + c2;
	}
	return out;
}

inline void fillAvoidanceSamples42FromCtrlPoints(
	const Eigen::Matrix3Xd &ctrl,
	Eigen::Ref<Eigen::Matrix3Xd> P_out)
{
	constexpr int M = 42;
	P_out.setZero();
	const int n = static_cast<int>(ctrl.cols());
	if (n < 6 || P_out.cols() != M)
		return;

	// fixed: first 3 and last 3 control points
	P_out.col(0) = ctrl.col(0);
	P_out.col(1) = ctrl.col(1);
	P_out.col(2) = ctrl.col(2);
	P_out.col(M - 3) = ctrl.col(n - 3);
	P_out.col(M - 2) = ctrl.col(n - 2);
	P_out.col(M - 1) = ctrl.col(n - 1);

	const int mid_cnt = M - 6;
	// polyline nodes from ctrl[2] .. ctrl[n-3]
	const int a0 = 2;
	const int a1 = n - 3;
	if (a1 <= a0)
		return;
	std::vector<double> seg_len;
	seg_len.reserve(static_cast<size_t>(a1 - a0));
	double total = 0.0;
	for (int i = a0; i < a1; ++i)
	{
		const double L = (ctrl.col(i + 1) - ctrl.col(i)).norm();
		seg_len.push_back(L);
		total += L;
	}
	if (!(std::isfinite(total)) || total < 1e-9)
		return;

	int seg_i = 0;
	double acc = 0.0;
	for (int m = 0; m < mid_cnt; ++m)
	{
		const double s = total * (static_cast<double>(m + 1) / static_cast<double>(mid_cnt + 1));
		while (seg_i < static_cast<int>(seg_len.size()) - 1 && acc + seg_len[static_cast<size_t>(seg_i)] < s)
		{
			acc += seg_len[static_cast<size_t>(seg_i)];
			seg_i++;
		}
		const double L = std::max(1e-12, seg_len[static_cast<size_t>(seg_i)]);
		const double t = (s - acc) / L;
		const Eigen::Vector3d p = (1.0 - t) * ctrl.col(a0 + seg_i) + t * ctrl.col(a0 + seg_i + 1);
		P_out.col(3 + m) = p;
	}
}

inline void fillCollisionSamples42FromCtrlPoints(
	const Eigen::Matrix3Xd &ctrl,
	Eigen::Ref<Eigen::Matrix3Xd> P_out)
{
	constexpr int M = 42;
	P_out.setZero();
	const int n = static_cast<int>(ctrl.cols());
	if (n < 21 || P_out.cols() != M)
		return;

	// fixed: first 3 control points
	P_out.col(0) = ctrl.col(0);
	P_out.col(1) = ctrl.col(1);
	P_out.col(2) = ctrl.col(2);
	// fixed: last 3 samples as control points #19..#21 (1-based) => indices 18..20
	P_out.col(M - 3) = ctrl.col(18);
	P_out.col(M - 2) = ctrl.col(19);
	P_out.col(M - 1) = ctrl.col(20);

	// Middle polyline: ctrl[2]..ctrl[12], then interpolate along ctrl[17]->ctrl[18] (P18->P19)
	std::vector<Eigen::Vector3d> nodes;
	nodes.reserve(32);
	for (int i = 2; i <= 12; ++i)
		nodes.push_back(ctrl.col(i));
	// include control point #18 (1-based) => index 17 as an intermediate node, even though #14..#18 are skipped as cost points
	nodes.push_back(ctrl.col(17));
	nodes.push_back(ctrl.col(18));
	if (nodes.size() < 2)
		return;

	std::vector<double> seg_len;
	seg_len.reserve(nodes.size() - 1);
	double total = 0.0;
	for (size_t i = 0; i + 1 < nodes.size(); ++i)
	{
		const double L = (nodes[i + 1] - nodes[i]).norm();
		seg_len.push_back(L);
		total += L;
	}
	if (!(std::isfinite(total)) || total < 1e-9)
		return;

	const int mid_cnt = M - 6;
	size_t seg_i = 0;
	double acc = 0.0;
	for (int m = 0; m < mid_cnt; ++m)
	{
		const double s = total * (static_cast<double>(m + 1) / static_cast<double>(mid_cnt + 1));
		while (seg_i + 1 < seg_len.size() && acc + seg_len[seg_i] < s)
		{
			acc += seg_len[seg_i];
			seg_i++;
		}
		const double L = std::max(1e-12, seg_len[seg_i]);
		const double t = (s - acc) / L;
		const Eigen::Vector3d p = (1.0 - t) * nodes[seg_i] + t * nodes[seg_i + 1];
		P_out.col(3 + m) = p;
	}
}

template <typename PlaneCandidatesVec>
inline std::vector<quadrotor_msgs::TrajectoryPlan> generateAvoidanceTrajectoryBatch(
	const geometry_msgs::PoseStamped &pose,
	const geometry_msgs::Vector3 &vel,
	const geometry_msgs::Vector3 &acc,
	const geometry_msgs::Vector3 & /*att*/, // kept for API symmetry; yaw is taken from pose
	const PlaneCandidatesVec &plane_candidates,
	const Eigen::Vector3d &goal,
	const Eigen::Vector3d &preferred_target,
	const Eigen::Vector3d &preferred_dir_unit,
	int batch_size = 72,
	double sample_radius_m = 3.0)
{
	(void)plane_candidates;

	if (batch_size <= 0)
		return {};
	if (batch_size > 72)
		batch_size = 72; // StudentPredictor default max

	const Eigen::Vector3d p0(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	const Eigen::Vector3d center = preferred_target;

	const double yaw = yawFromQuat(pose.pose.orientation);
	const Eigen::Vector2d yaw_fwd_world(std::cos(yaw), std::sin(yaw));

	Eigen::Vector2d v_world(vel.x, vel.y);
	Eigen::Vector2d forward_world = v_world;
	if (!forward_world.allFinite() || forward_world.norm() < 1e-3)
		forward_world = yaw_fwd_world;
	else
		forward_world.normalize();

	const Eigen::Matrix2d R_wb = rotWorldToBody2DFromForward(forward_world);
	const Eigen::Matrix2d R_bw = R_wb.transpose().eval();

	// Build sampled geometry offsets (world) around preferred_target.
	std::vector<Eigen::Vector2d> targets_world;
	targets_world.resize(batch_size);
	targets_world[0] = center.head<2>();

	thread_local std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<double> uni01(0.0, 1.0);
	std::uniform_real_distribution<double> uniAng(0.0, 2.0 * M_PI);
	for (int i = 1; i < batch_size; ++i)
	{
		const double u = uni01(rng);
		const double r = sample_radius_m * std::sqrt(u);
		const double th = uniAng(rng);
		Eigen::Vector2d off(r * std::cos(th), r * std::sin(th));
		targets_world[i] = center.head<2>() + off;
	}

	// Prepare StudentPredictor raw inputs: batch_size x 14
	std::vector<float> input_raw;
	input_raw.resize(static_cast<size_t>(batch_size) * 14u, 0.0f);

	// State in world
	const Eigen::Vector2d a_world(acc.x, acc.y);
	Eigen::Vector2d v_local = R_wb * v_world;
	v_local.y() = 0.0; // enforce vy=0 in student input
	const Eigen::Vector2d a_local = R_wb * a_world;

	for (int i = 0; i < batch_size; ++i)
	{
		const Eigen::Vector2d geom_local = R_wb * (targets_world[i] - p0.head<2>());
		const size_t base = static_cast<size_t>(i) * 14u;
		// New Student input ordering: tm, sp, sv, sa, yv, go, gv
		// tm: task_mask (avoidance -> [0,1])
		input_raw[base + 0] = 0.0f;
		input_raw[base + 1] = 1.0f;
		// sp: start_pos (after translate+rotate) -> (0,0)
		input_raw[base + 2] = 0.0f;
		input_raw[base + 3] = 0.0f;
		// sv: start_vel
		input_raw[base + 4] = static_cast<float>(v_local.x());
		input_raw[base + 5] = static_cast<float>(v_local.y());
		// sa: start_acc
		input_raw[base + 6] = static_cast<float>(a_local.x());
		input_raw[base + 7] = static_cast<float>(a_local.y());
		// yv: yaw_vector (use real yaw) -> (cos, sin)
		input_raw[base + 8] = static_cast<float>(yaw_fwd_world.x());
		input_raw[base + 9] = static_cast<float>(yaw_fwd_world.y());
		// go: geometry_offset
		input_raw[base + 10] = static_cast<float>(geom_local.x());
		input_raw[base + 11] = static_cast<float>(geom_local.y());
		// gv: guide_vector (fixed for avoidance)
		input_raw[base + 12] = 1.0f;
		input_raw[base + 13] = 0.0f;
	}

	// Run Student model (preferred). If TRT isn't ready, fall back to straight-line bspline per sample.
	StudentPredictor::RemappedBatch remapped;
	bool model_ok = false;
	try
	{
		trt::TrtManager &mgr = trt::TrtManager::instance();
		if (mgr.isReady())
		{
			StudentPredictor &student = mgr.student();
			student.predict_remap(input_raw, remapped, batch_size);
			// remapped.control_points_padded is padded to (B,21,2)
			model_ok = (remapped.control_points_padded.size() >= static_cast<size_t>(batch_size) * 21u * 2u)
					   && (remapped.lengths.size() == static_cast<size_t>(batch_size));
		}
	}
	catch (const std::exception &e)
	{
		ROS_WARN("generateAvoidanceTrajectoryBatch: student model unavailable (%s)", e.what());
		model_ok = false;
	}

	std::vector<quadrotor_msgs::TrajectoryPlan> plans;
	plans.resize(batch_size);

	for (int i = 0; i < batch_size; ++i)
	{
		const Eigen::Vector3d p1(targets_world[i].x(), targets_world[i].y(), center.z());
		if (!model_ok)
		{
			// Fallback (internal candidate): clamped BSPLINE ctrl points along the straight line.
			plans[i] = buildStraightLineBsplineCtrlPlan(p0, p1, preferred_dir_unit, quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);
			continue;
		}

		std::array<Eigen::Vector2d, SAMPLE_FORWARD_BSPLINE_CTRL_N> ctrl_xy_world;
		// remapped.control_points_padded is B x 21 x 2 flattened. For avoidance samples
		// remapped.lengths[i] should be 18. If not, fall back to straight-line plan.
		const int reported_len = (model_ok && remapped.lengths.size() == static_cast<size_t>(batch_size)) ? remapped.lengths[i] : 0;
		if (reported_len < SAMPLE_FORWARD_BSPLINE_CTRL_N)
		{
			// Unexpected length: fall back to straight-line control plan
			plans[i] = buildStraightLineBsplineCtrlPlan(p0, p1, preferred_dir_unit, quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);
			continue;
		}

		for (int j = 0; j < SAMPLE_FORWARD_BSPLINE_CTRL_N; ++j)
		{
			const size_t base_idx = static_cast<size_t>(i) * 21u + static_cast<size_t>(j);
			const size_t fidx = base_idx * 2u;
			const Eigen::Vector2d cp_local(remapped.control_points_padded[fidx + 0], remapped.control_points_padded[fidx + 1]);
			const Eigen::Vector2d cp_world = p0.head<2>() + R_bw * cp_local;
			ctrl_xy_world[j] = cp_world;
		}
		// Enforce clamped endpoints semantics: start at p0, end at sampled geometry target.
		ctrl_xy_world.front() = p0.head<2>();
		ctrl_xy_world.back() = p1.head<2>();
		fillBsplinePlanFromWorldCtrlPoints(plans[i], p0, p1, preferred_dir_unit, ctrl_xy_world,
										quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);
	}

	return plans;
}

inline quadrotor_msgs::TrajectoryPlan makeTwoPointMinJerkPlan(
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &dir_unit,
	int mode)
{
	// True minimum-jerk quintic (zero v/a at endpoints).
	// Compute an optimal duration using the OBVP-derived polynomial root
	// solver. If the solver fails or returns non-positive T, fall back to
	// the fixed duration.
	double T = SAMPLE_FORWARD_FIXED_DURATION_S;
	{
		// OBVP solver (only X/Y considered) copied locally to avoid extra
		// include dependencies. Uses companion-matrix root finding.
		double optimal_T = 0.0;
		double optimal_cost = 100000.0;
		Eigen::Vector3d delta_pos = p1 - p0;
		delta_pos.z() = 0.0;
		Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
		start_vel.z() = 0.0;

		const double c0 = -9.0 * delta_pos.squaredNorm();
		const double c1 = 12.0 * delta_pos.dot(start_vel);
		const double c2 = -3.0 * start_vel.squaredNorm();
		const double c3 = 0.0;

		Eigen::Matrix4d companion;
		companion << 0, 0, 0, -c0,
					 1, 0, 0, -c1,
					 0, 1, 0, -c2,
					 0, 0, 1, -c3;

		Eigen::EigenSolver<Eigen::Matrix4d> solver(companion);
		auto roots = solver.eigenvalues();
		for (int ri = 0; ri < roots.size(); ++ri) {
			const double rval = roots(ri).real();
			const double rimg = roots(ri).imag();
			if (std::abs(rimg) >= 1e-6 || rval <= 1e-6) continue;
			const double cost = rval
				+ 3.0 * start_vel.squaredNorm() / rval
				- 6.0 * delta_pos.dot(start_vel) / (rval * rval)
				+ 3.0 * delta_pos.squaredNorm() / (rval * rval * rval);
			if (cost < optimal_cost) {
				optimal_cost = cost;
				optimal_T = rval;
			}
		}

		if (optimal_cost < 100000.0 && optimal_T > 1e-6) {
			T = optimal_T;
		}
	}
	const int M = 240; // dense time samples
	std::vector<Eigen::Vector3d> dense_pts;
	std::vector<double> dense_t;
	dense_pts.reserve(M);
	dense_t.reserve(M);

	for (int k = 0; k < M; ++k)
	{
		const double t = (static_cast<double>(k) / static_cast<double>(M - 1)) * T;
		double s = 0.0;
		if (T > 1e-9)
			s = clamp01(t / T);
		const double s2 = s * s;
		const double s3 = s2 * s;
		const double s4 = s3 * s;
		const double s5 = s4 * s;
		const double h = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
		const Eigen::Vector3d p = p0 + (p1 - p0) * h;
		dense_pts.push_back(p);
		dense_t.push_back(t);
	}

	std::vector<Eigen::Vector3d> pts;
	std::vector<double> ts;
	resampleMethod2ArcLength(dense_pts, dense_t, 1.0, pts, ts);
	if (pts.size() < 2)
	{
		pts.clear();
		ts.clear();
		pts.push_back(p0);
		ts.push_back(0.0);
		// insert time-midpoint sample
		const double t_mid = 0.5 * T;
		const double s_mid = (T > 1e-9) ? clamp01(t_mid / T) : 0.5;
		const double s2_mid = s_mid * s_mid;
		const double s3_mid = s2_mid * s_mid;
		const double s4_mid = s3_mid * s_mid;
		const double s5_mid = s4_mid * s_mid;
		const double h_mid = 10.0 * s3_mid - 15.0 * s4_mid + 6.0 * s5_mid;
		pts.push_back(p0 + (p1 - p0) * h_mid);
		ts.push_back(t_mid);
		pts.push_back(p1);
		ts.push_back(T);
	}
	else if (pts.size() == 2)
	{
		// If only start and end remain after resampling, insert the trajectory midpoint
		// defined at the temporal midpoint (t = T/2) using the same min-jerk interpolation.
		const double t_mid = 0.5 * T;
		const double s_mid = (T > 1e-9) ? clamp01(t_mid / T) : 0.5;
		const double s2_mid = s_mid * s_mid;
		const double s3_mid = s2_mid * s_mid;
		const double s4_mid = s3_mid * s_mid;
		const double s5_mid = s4_mid * s_mid;
		const double h_mid = 10.0 * s3_mid - 15.0 * s4_mid + 6.0 * s5_mid;
		Eigen::Vector3d p_mid = p0 + (p1 - p0) * h_mid;
		// insert between first and last
		pts.insert(pts.begin() + 1, p_mid);
		ts.insert(ts.begin() + 1, t_mid);
	}
	return buildWaypointsPlanFromSamples(mode, pts, ts, dir_unit);
}

inline quadrotor_msgs::TrajectoryPlan buildStraightLineBsplineCtrlPlan(
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &dir_unit,
	int mode)
{
	std::array<Eigen::Vector2d, SAMPLE_FORWARD_BSPLINE_CTRL_N> ctrl_xy_world;
	for (int j = 0; j < SAMPLE_FORWARD_BSPLINE_CTRL_N; ++j)
	{
		const double a = static_cast<double>(j) / static_cast<double>(SAMPLE_FORWARD_BSPLINE_CTRL_N - 1);
		const Eigen::Vector3d p = p0 + a * (p1 - p0);
		ctrl_xy_world[j] = p.head<2>();
	}
	quadrotor_msgs::TrajectoryPlan out;
	fillBsplinePlanFromWorldCtrlPoints(out, p0, p1, dir_unit, ctrl_xy_world, mode);
	return out;
}

// Forward declaration: evalBsplineGeneric is defined later in this file
// but getAvoidanceBsplineMatrices() needs to call it here.
inline double evalBsplineGeneric(const std::vector<double>& ctrl, int p, double u);

// Precompute and cache the clamped uniform B-spline basis matrix for
// avoidance trajectories: 18 control points, degree 5, 42 samples.
struct PreAvoidanceMatrices {
	Eigen::MatrixXd Bt_pos;
	Eigen::MatrixXd Bt_vel; // d/du
	Eigen::MatrixXd Bt_acc; // d^2/du^2
};

inline const PreAvoidanceMatrices& getAvoidanceBsplineMatrices()
{
	static const PreAvoidanceMatrices mats = []() {
		constexpr int M = 42;
		constexpr int CtrlN = SAMPLE_FORWARD_BSPLINE_CTRL_N;
		static_assert(CtrlN == 18, "Avoidance expects 18 control points");
		static_assert(SAMPLE_FORWARD_BSPLINE_DEGREE == 5, "Avoidance expects degree-5 B-spline");

		PreAvoidanceMatrices res;
		res.Bt_pos.resize(CtrlN, M);
		res.Bt_vel.resize(CtrlN, M);
		res.Bt_acc.resize(CtrlN, M);

		constexpr int p = SAMPLE_FORWARD_BSPLINE_DEGREE;

		// Helper to get derivative CPs (same as collision version)
		auto get_deriv_cps = [&](const std::vector<double>& P, int deg) {
			int n = P.size();
			std::vector<double> Q(n - 1);
			int spans = n - deg;
			auto knot = [&](int i) -> double {
				if (i <= deg) return 0.0;
				if (i >= n) return static_cast<double>(spans);
				return static_cast<double>(i - deg);
			};
			
			for (int i = 0; i < n - 1; ++i) {
				double U_next = knot(i + deg + 1);
				double U_curr = knot(i + 1);
				double den = U_next - U_curr;
				double val = 0.0;
				if (std::abs(den) > 1e-9) {
					val = static_cast<double>(deg) * (P[i+1] - P[i]) / den;
				}
				Q[i] = val;
			}
			return Q;
		};

		for (int j = 0; j < CtrlN; ++j)
		{
			std::vector<double> P(CtrlN, 0.0);
			P[j] = 1.0;

			// 1st derivative CPs (size CtrlN - 1)
			std::vector<double> Q = get_deriv_cps(P, p);
			// 2nd derivative CPs (size CtrlN - 2)
			std::vector<double> R = get_deriv_cps(Q, p - 1);

			for (int k = 0; k < M; ++k)
			{
				const double u = (static_cast<double>(k) / static_cast<double>(M - 1)) *
								 static_cast<double>(CtrlN - SAMPLE_FORWARD_BSPLINE_DEGREE);
				res.Bt_pos(j, k) = evalBsplineGeneric(P, p, u);
				res.Bt_vel(j, k) = evalBsplineGeneric(Q, p - 1, u);
				res.Bt_acc(j, k) = evalBsplineGeneric(R, p - 2, u);
			}
		}
		
		PreAvoidanceMatrices final_res;
		final_res.Bt_pos = res.Bt_pos.transpose(); // (M x CtrlN)
		final_res.Bt_vel = res.Bt_vel.transpose();
		final_res.Bt_acc = res.Bt_acc.transpose();
		return final_res;
	}();
	return mats;
}

// ---------------------------------------------------------
// Helper: Evaluate generic B-spline (clamped uniform) of given degree
// Used for precomputing derivative basis matrices.
// ---------------------------------------------------------
inline double evalBsplineGeneric(
	const std::vector<double>& ctrl, 
	int p, 
	double u)
{
	const int n = static_cast<int>(ctrl.size());
	const int spans = n - p;

	if (n <= p) return 0.0;
	if (u <= 0.0) return ctrl.front();
	if (u >= static_cast<double>(spans)) return ctrl.back();

	int k = p + static_cast<int>(std::floor(u));
	if (k < p) k = p;
	if (k > n - 1) k = n - 1;

	// De Boor
	std::vector<double> d(p + 1);
	for (int j = 0; j <= p; ++j) {
		d[j] = ctrl[k - p + j];
	}

	// Knot function for clamped uniform: 
	// [0...0 (p+1 times), 1, 2, ..., spans, spans...spans (p+1 times)]
	auto knot = [&](int i) -> double {
		if (i <= p) return 0.0;
		if (i >= spans + p) return static_cast<double>(spans); 
		// The internal knots are 1, 2, ..., spans. 
		// The knot vector has length n + p + 1. 
		// Indices: 0..p -> 0
		// p+1 -> 1
		// ...
		// n -> spans
		// n+1..n+p -> spans
		if (i >= n) return static_cast<double>(spans); // Clamp at end
		return static_cast<double>(i - p);
	};

	for (int r = 1; r <= p; ++r) {
		for (int j = p; j >= r; --j) {
			int idx_k = k - p + j; // corresponds to index in knot vector for interval start
			// alpha = (u - U[idx_k]) / (U[idx_k + p + 1 - r] - U[idx_k])
			double U_i = knot(idx_k);
			double U_ip = knot(idx_k + p + 1 - r);
			double denom = U_ip - U_i;
			double alpha = 0.0;
			if (std::abs(denom) > 1e-9) {
				alpha = (u - U_i) / denom;
			}
			d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
		}
	}
	return d[p];
}

// Precompute and cache:
// 1. Position basis Bt_pre (M x N)
// 2. Velocity (1st deriv wrt u) basis Bt_pre_vel (M x N)
// 3. Acceleration (2nd deriv wrt u) basis Bt_pre_acc (M x N)
struct PreCollisionMatrices {
	Eigen::MatrixXd Bt_pos;
	Eigen::MatrixXd Bt_vel; // d/du
	Eigen::MatrixXd Bt_acc; // d^2/du^2
};

inline const PreCollisionMatrices& getCollisionPreBsplineMatrices()
{
	static const PreCollisionMatrices mats = []() {
		constexpr int Mpre = 28;
		constexpr int CtrlNpre = 16;
		constexpr int p = 5; // Degree
		
		PreCollisionMatrices res;
		res.Bt_pos.resize(CtrlNpre, Mpre);
		res.Bt_vel.resize(CtrlNpre, Mpre);
		res.Bt_acc.resize(CtrlNpre, Mpre);

		// Helper to get derivative CPs
		// Input P (size n), returns Q (size n-1)
		auto get_deriv_cps = [&](const std::vector<double>& P, int deg) {
			int n = P.size();
			std::vector<double> Q(n - 1);
			int spans = n - deg; // max knot value
			// Knots for degree 'deg':
			auto knot = [&](int i) -> double {
				if (i <= deg) return 0.0;
				if (i >= n) return static_cast<double>(spans);
				return static_cast<double>(i - deg);
			};
			
			for (int i = 0; i < n - 1; ++i) {
				// Q_i = deg * (P_{i+1} - P_i) / (U_{i+deg+1} - U_{i+1})
				double U_next = knot(i + deg + 1);
				double U_curr = knot(i + 1);
				double den = U_next - U_curr;
				double val = 0.0;
				if (std::abs(den) > 1e-9) {
					val = static_cast<double>(deg) * (P[i+1] - P[i]) / den;
				}
				Q[i] = val;
			}
			return Q;
		};

		// Construct basis by probing each control point
		for (int j = 0; j < CtrlNpre; ++j)
		{
			// 1. One-hot control points
			std::vector<double> P(CtrlNpre, 0.0);
			P[j] = 1.0;

			// 2. First derivative CPs (size 15, deg 4)
			std::vector<double> Q = get_deriv_cps(P, p);

			// 3. Second derivative CPs (size 14, deg 3)
			std::vector<double> R = get_deriv_cps(Q, p - 1);

			// 4. Sample along u
			for (int k = 0; k < Mpre; ++k)
			{
				double u = (static_cast<double>(k) / static_cast<double>(Mpre - 1)) *
						   static_cast<double>(CtrlNpre - p); // [0, 11]

				// Pos: eval P, deg 5
				res.Bt_pos(j, k) = evalBsplineGeneric(P, p, u);
				// Vel: eval Q, deg 4
				res.Bt_vel(j, k) = evalBsplineGeneric(Q, p - 1, u);
				// Acc: eval R, deg 3
				res.Bt_acc(j, k) = evalBsplineGeneric(R, p - 2, u);
			}
		}
		
		// Transpose to (M x N) layout for convenient (M x N) * (N x Batch) multiplication
		res.Bt_pos.transposeInPlace();
		res.Bt_vel.transposeInPlace();
		res.Bt_acc.transposeInPlace(); // Now (Mpre x CtrlNpre)

		return res;
	}();
	return mats;
}

// Templated evaluation for clamped uniform B-spline supporting arbitrary control-point counts.
template <size_t N>
inline Eigen::Vector2d evalClampedUniformBspline2D(
	const std::array<Eigen::Vector2d, N> &ctrl,
	double u)
{
	constexpr int p = SAMPLE_FORWARD_BSPLINE_DEGREE;
	const int n = static_cast<int>(N);
	const int spans = n - p;

	if (!std::isfinite(u)) u = 0.0;
	if (u <= 0.0) return ctrl.front();
	if (u >= static_cast<double>(spans)) return ctrl.back();

	int k = p + static_cast<int>(std::floor(u));
	if (k < p) k = p;
	if (k > n - 1) k = n - 1;

	Eigen::Vector2d d[p + 1];
	for (int j = 0; j <= p; ++j)
	{
		d[j] = ctrl[k - p + j];
	}

	auto knot = [&](int i) -> double {
		if (i <= p) return 0.0;
		if (i >= spans) return static_cast<double>(spans);
		return static_cast<double>(i - p);
	};

	for (int r = 1; r <= p; ++r)
	{
		for (int j = p; j >= r; --j)
		{
			const int idx = k - p + j;
			const double denom = knot(idx + p + 1 - r) - knot(idx);
			double alpha = 0.0;
			if (std::abs(denom) > 1e-12)
				alpha = (u - knot(idx)) / denom;
			alpha = clamp(alpha, 0.0, 1.0);
			d[static_cast<size_t>(j)] = (1.0 - alpha) * d[static_cast<size_t>(j - 1)] + alpha * d[static_cast<size_t>(j)];
		}
	}

	return d[static_cast<size_t>(p)];
}

template <typename PlaneCandidatesVec>
inline quadrotor_msgs::TrajectoryPlan generateAvoidanceTrajectory(
	const geometry_msgs::PoseStamped &pose,
	const geometry_msgs::Vector3 &vel,
	const geometry_msgs::Vector3 &acc,
	const geometry_msgs::Vector3 &att,
	const PlaneCandidatesVec &plane_candidates,
	const Eigen::Vector3d &goal,
	const Eigen::Vector3d &preferred_target,
	const Eigen::Vector3d &preferred_dir_unit,
	const Config &config)
{
	(void)att;

	const Eigen::Vector3d p0(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	const Eigen::Vector3d p1 = goal;
	const double dist = (p1 - p0).norm();

	if (std::isfinite(dist)) {
		double rms = 0.0, vmax = 0.0;
		const bool violated = straightLineRmsViolationExceeds(p0, p1, plane_candidates, rms, vmax);
		if (!violated) {
			Eigen::Vector3d dir = preferred_dir_unit;
			dir.z() = 0.0;
			if (dir.head<2>().norm() > 1e-6) dir.normalize();
			else {
				Eigen::Vector3d d = (p1 - p0); d.z() = 0.0;
				if (d.head<2>().norm() > 1e-6) dir = d.normalized();
				else dir = Eigen::Vector3d(1.0, 0.0, 0.0);
			}
			return makeTwoPointMinJerkPlan(p0, p1, dir, quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);
		}
	}
	// Straight line is unsafe (or goal too far): generate a batch of candidate avoidance trajectories
	// around preferred_target (XY disk), and choose the minimum-cost candidate.
	const auto candidates = generateAvoidanceTrajectoryBatch(pose, vel, acc, att, plane_candidates,
										goal, preferred_target, preferred_dir_unit,
										72, 3.0);
	if (candidates.empty())
		return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);

	// Read kinematic limits from YAML-loaded ROS params
	double max_vel = 4.0, max_acc = 4.0;
	readMaxVelAccFromRosParams(max_vel, max_acc);

	// Pre-build corridor group matrices for vectorized violation evaluation
	std::vector<Eigen::MatrixXd> group_normals;
	std::vector<Eigen::VectorXd> group_ds;
	buildCorridorGroupMatrices(plane_candidates, group_normals, group_ds);

	constexpr int M = 42; // samples along the spline
	// Avoidance duration fixed to 4s.
	const double total_T = 4.0;
	const double dt = total_T / static_cast<double>(M - 1);

	Eigen::Vector2d pref_dir(preferred_dir_unit.x(), preferred_dir_unit.y());
	if (!pref_dir.allFinite() || pref_dir.norm() < 1e-6) pref_dir = Eigen::Vector2d(1.0, 0.0);
	else pref_dir.normalize();

	// Scoring weights read directly from provided Config
	const double w_collision = config.w_collision;
	const double w_kin = config.w_kin;
	const double w_dir = config.w_dir;
	const double w_end = config.w_end;

	ROS_DEBUG("Avoidance scoring weights: w_collision=%.3f, w_kin=%.3f, w_dir=%.3f, w_end=%.3f",
			  w_collision,  w_kin,  w_dir,  w_end);

	int best_i = 0;

	// Vectorized scoring buffers (avoid per-candidate scalar direction/end computations)
	const int N = static_cast<int>(candidates.size());
	Eigen::Array<bool, Eigen::Dynamic, 1> valid_candidate(N);
	valid_candidate.setConstant(false);
	Eigen::ArrayXd coll_norm_vec = Eigen::ArrayXd::Zero(N);
	Eigen::ArrayXd kin_cost_vec = Eigen::ArrayXd::Zero(N);
	Eigen::Matrix<double, 2, Eigen::Dynamic> end_xy_mat(2, N);
	Eigen::Matrix<double, 2, Eigen::Dynamic> back_xy_mat(2, N);
	end_xy_mat.setZero();
	back_xy_mat.setZero();

	// direction uses P(T) - P(T-1.3s)
	const double lookback_s = 1.3;
	const int lookback_steps = static_cast<int>(std::lround(lookback_s / dt));
	const int k0 = clamp((M - 1) - lookback_steps, 0, M - 1);

	// ---- Avoidance: use B-spline basis to sample 42 points for ALL candidates ----
	// Build control point matrices (CtrlN x N), then sample via:
	//   X = Bt * Cx, Y = Bt * Cy
	// These sampled points are used for BOTH collision cost and kinematic cost.
	if (N > 0)
	{
		constexpr int CtrlN = SAMPLE_FORWARD_BSPLINE_CTRL_N;
		const auto &avoid_mats = getAvoidanceBsplineMatrices();
		const Eigen::MatrixXd &Bt = avoid_mats.Bt_pos;
		const Eigen::MatrixXd &Bt_vel = avoid_mats.Bt_vel;
		const Eigen::MatrixXd &Bt_acc = avoid_mats.Bt_acc;

		Eigen::MatrixXd Cx(CtrlN, N);
		Eigen::MatrixXd Cy(CtrlN, N);
		Cx.setZero();
		Cy.setZero();
		Eigen::VectorXd z0(N);
		z0.setZero();

		for (int i = 0; i < N; ++i)
		{
			const auto &plan = candidates[i];
			if (plan.waypoints.size() < CtrlN)
				continue;
			valid_candidate(i) = true;
			z0(i) = plan.waypoints.front().z;
			for (int j = 0; j < CtrlN; ++j)
			{
				Cx(j, i) = plan.waypoints[j].x;
				Cy(j, i) = plan.waypoints[j].y;
			}
		}

		const Eigen::MatrixXd X = Bt * Cx; // (M x N)
		const Eigen::MatrixXd Y = Bt * Cy; // (M x N)

		// Collision cost: pack into P_big (3 x (M*N))
		if (!group_normals.empty())
		{
			Eigen::Matrix3Xd P_big(3, M * N);
			P_big.setZero();
			for (int i = 0; i < N; ++i)
			{
				P_big.block(0, i * M, 1, M) = X.col(i).transpose().eval();
				P_big.block(1, i * M, 1, M) = Y.col(i).transpose().eval();
				P_big.block(2, i * M, 1, M).setConstant(z0(i));
			}
			coll_norm_vec = corridorViolationCostBatchVectorized(P_big, M, N, group_normals, group_ds);
		}

		// Kinematic cost: velocities/accelerations from sampled points
		const double spans = static_cast<double>(CtrlN - SAMPLE_FORWARD_BSPLINE_DEGREE);
		const double scale_v = spans / total_T;
		const double scale_a = scale_v * scale_v;

		const Eigen::MatrixXd Vx = (Bt_vel * Cx) * scale_v;
		const Eigen::MatrixXd Vy = (Bt_vel * Cy) * scale_v;
		const Eigen::MatrixXd Ax = (Bt_acc * Cx) * scale_a;
		const Eigen::MatrixXd Ay = (Bt_acc * Cy) * scale_a;

		const Eigen::ArrayXd max_abs_vx = Vx.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
		const Eigen::ArrayXd max_abs_vy = Vy.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
		const Eigen::ArrayXd max_abs_ax = Ax.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
		const Eigen::ArrayXd max_abs_ay = Ay.cwiseAbs().colwise().maxCoeff().transpose().eval().array();

		Eigen::ArrayXd v_cost = Eigen::ArrayXd::Zero(N);
		v_cost += ((max_abs_vx - max_vel) / max_vel).max(0.0);
		v_cost += ((max_abs_vy - max_vel) / max_vel).max(0.0);
		Eigen::ArrayXd a_cost = Eigen::ArrayXd::Zero(N);
		a_cost += ((max_abs_ax - max_acc) / max_acc).max(0.0);
		a_cost += ((max_abs_ay - max_acc) / max_acc).max(0.0);
		kin_cost_vec = v_cost + a_cost;

		// Endpoint/back samples for vectorized direction/endpoint scoring
		end_xy_mat.row(0) = X.row(M - 1);
		end_xy_mat.row(1) = Y.row(M - 1);
		back_xy_mat.row(0) = X.row(k0);
		back_xy_mat.row(1) = Y.row(k0);
	}

	// Vectorized Scheme A direction/end costs + total cost, then argmin
	if (N > 0)
	{
		const Eigen::ArrayXd tan_x = (end_xy_mat.row(0).array() - back_xy_mat.row(0).array());
		const Eigen::ArrayXd tan_y = (end_xy_mat.row(1).array() - back_xy_mat.row(1).array());
		const Eigen::ArrayXd tan_norm = (tan_x.square() + tan_y.square()).sqrt();
		const Eigen::ArrayXd valid = (tan_norm > 1e-6).cast<double>();
		Eigen::ArrayXd dir_cost = Eigen::ArrayXd::Ones(N);
		if ((valid > 0.0).any())
		{
			Eigen::ArrayXd tx = tan_x / tan_norm.max(1e-12);
			Eigen::ArrayXd ty = tan_y / tan_norm.max(1e-12);
			Eigen::ArrayXd dot = (tx * pref_dir.x() + ty * pref_dir.y());
			dot = dot.max(-1.0).min(1.0);
			dir_cost = (1.0 - dot) * valid + dir_cost * (1.0 - valid);
		}

		const Eigen::ArrayXd ex = end_xy_mat.row(0).array() - preferred_target.x();
		const Eigen::ArrayXd ey = end_xy_mat.row(1).array() - preferred_target.y();
		const double pref_perp_x = -pref_dir.y();
		const double pref_perp_y = pref_dir.x();
		const Eigen::ArrayXd end_long = (ex * pref_dir.x() + ey * pref_dir.y()).abs();
		const Eigen::ArrayXd end_lat = (ex * pref_perp_x + ey * pref_perp_y).abs();
		const Eigen::ArrayXd end_cost = w_dir * end_lat + w_end * end_long;

		Eigen::ArrayXd total_cost = w_collision * coll_norm_vec + w_kin * kin_cost_vec + w_dir * dir_cost + end_cost;
		total_cost = valid_candidate.select(total_cost, Eigen::ArrayXd::Constant(N, std::numeric_limits<double>::infinity()));
		Eigen::Index idx = 0;
		total_cost.minCoeff(&idx);
		best_i = static_cast<int>(idx);
	}

	// IMPORTANT: candidates are BSPLINE ctrl plans for scoring. Convert only the final selected plan
	// to discrete waypoints for the backend (REP_WAYPOINTS).
	const auto &best = candidates[best_i];
	// printf("generateAvoidanceTrajectory: selected candidate %d / %d", best_i, N);
	if (best.waypoints.size() >= SAMPLE_FORWARD_BSPLINE_CTRL_N)
	{
		return convertClampedUniformBsplineCtrlPlanToWaypointsMethod2(best, preferred_dir_unit, SAMPLE_FORWARD_FIXED_DURATION_S);
		// printf("generateAvoidanceTrajectory: converted to waypoints plan with %zu waypoints", best.waypoints.size());
	}
	return best;
}

template <typename PlaneCandidatesVec>
inline quadrotor_msgs::TrajectoryPlan generateMixedTrajectory(
	const geometry_msgs::PoseStamped &pose,
	const geometry_msgs::Vector3 &vel,
	const geometry_msgs::Vector3 &acc,
	const geometry_msgs::Vector3 &att,
	const PlaneCandidatesVec &plane_candidates,
	const Eigen::Vector3d &goal,
	const Eigen::Vector3d &preferred_target,
	const Eigen::Vector3d &preferred_dir_unit,
	const Config &config,
	const Eigen::Vector3d &plane_n_unit,
	const double plane_d,
	const double plane_width,
	const Eigen::Vector3d &plane_point,
	quadrotor_msgs::CollisionTrajectory *out_collision_traj = nullptr,
	bool *out_has_collision_traj = nullptr)
{
	(void)att;
	ROS_INFO("plane_d_unit: (%.3f, %.3f, %.3f), plane_d: %.3f",
			 plane_n_unit.x(), plane_n_unit.y(), plane_n_unit.z(), plane_d);
	// quick straight-line fallback (same logic as generateAvoidanceTrajectory)
	const Eigen::Vector3d p0(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	const Eigen::Vector3d p_goal = goal;
	const double dist = (p_goal - p0).norm();
	if (std::isfinite(dist)) {
		double rms = 0.0, vmax = 0.0;
		const bool violated = straightLineRmsViolationExceeds(p0, p_goal, plane_candidates, rms, vmax);
		if (!violated) {
			Eigen::Vector3d dir = preferred_dir_unit;
			dir.z() = 0.0;
			if (dir.head<2>().norm() > 1e-6) dir.normalize();
			else {
				Eigen::Vector3d d = (p_goal - p0); d.z() = 0.0;
				if (d.head<2>().norm() > 1e-6) dir = d.normalized();
				else dir = Eigen::Vector3d(1.0, 0.0, 0.0);
			}
			return makeTwoPointMinJerkPlan(p0, p_goal, dir, quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);
		}
	}

	// Scoring weights read directly from provided Config
	const double w_collision = config.w_collision;
	const double w_kin = config.w_kin;
	const double w_dir = config.w_dir;
	const double w_end = config.w_end;
	if (out_has_collision_traj) *out_has_collision_traj = false;

	// NOTE: For mixed (collision) mode, flip the *normal component* only.
	// Here we follow the convention used by the caller: the 1st component is the normal direction,
	// while the 2nd/3rd components are tangential and must remain unchanged.
	// Any CollisionEvent plane equation generated by this path must use this flipped normal.
	Eigen::Vector3d plane_n_unit_flipped = plane_n_unit;
	plane_n_unit_flipped.x() = -plane_n_unit_flipped.x();
	plane_n_unit_flipped.y() = -plane_n_unit_flipped.y();

	// Read max collision velocity from provided Config (loaded from YAML)
	double max_collision_velocity = config.max_collision_velocity;

	// We will use plane_n_unit_flipped/plane_d/plane_width/max_collision_velocity below.

	const double yaw = yawFromQuat(pose.pose.orientation);
	const Eigen::Vector2d yaw_fwd_world(std::cos(yaw), std::sin(yaw));
	Eigen::Vector2d v_world(vel.x, vel.y);
	Eigen::Vector2d forward_world = v_world;
	if (!forward_world.allFinite() || forward_world.norm() < 1e-3)
		forward_world = yaw_fwd_world;
	else
		forward_world.normalize();
	const Eigen::Matrix2d R_wb = rotWorldToBody2DFromForward(forward_world);
	const Eigen::Matrix2d R_bw = R_wb.transpose().eval();
	const Eigen::Vector2d a_world(acc.x, acc.y);
	Eigen::Vector2d v_local = R_wb * v_world;
	v_local.y() = 0.0; // enforce vy=0 in student input
	const Eigen::Vector2d a_local = R_wb * a_world;
	{
		// 1. 基础数据准备
		const Eigen::Vector3d n = plane_n_unit_flipped; // 法线指向可行区域
		const double dist_R = n.dot(p0) + plane_d;      // 起点离墙的垂直距离

		Eigen::Vector3d baseline_pt;
		// 虚拟目标点
		Eigen::Vector3d p_virtual;

		if (dist_R > 1e-6)
		{
			// 2. 将起点和终点都投影到墙面上 (关键步骤)
			// 我们需要知道终点相对于起点，在"墙面平行方向"上偏哪边
			Eigen::Vector3d p0_proj = p0 - dist_R * n;
			
			// 计算终点的投影 (注意：preferred_target 包含了侧向信息)
			double dist_target = n.dot(preferred_target) + plane_d;
			Eigen::Vector3d target_proj = preferred_target - dist_target * n;

			// 3. 提取墙面切向向量 (Wall Tangent Vector)
			Eigen::Vector3d vec_on_wall = target_proj - p0_proj;
			double lateral_dist = vec_on_wall.norm();

			// 4. 步长截断 (Clamping Logic)
			// 如果终点很远，或者是U形弯，我们不能盲目瞄准那个投影点（太远会撞角）
			// 同时也解决了 U 形弯 dir 向量失效的问题
			
			// 设定安全步长为 1.0 倍的离墙距离 (产生约 26度 切角)
			double safe_step = dist_R * 1.0; 

			// 如果终点侧向距离太远，或者仅仅是想绕障，我们只走 safe_step 这么远
			// 如果 lateral_dist 很小 (比如 < 1e-3)，说明终点就在正后方，那就只能垂直打
			if (lateral_dist > 1e-4) {
				vec_on_wall = vec_on_wall.normalized() * std::min(lateral_dist, safe_step);
			} else {
				// 纯垂直回弹情况
				vec_on_wall = Eigen::Vector3d::Zero();
			}

			// 5. 构造墙上的虚拟目标点
			// 这个点既保留了终点的"左右方向"，又限制了"距离"
			Eigen::Vector3d p_virtual_on_wall = p0_proj + vec_on_wall;

			// 额外：在墙面上构造带法向偏移的虚拟点 p_virtual（还不使用，只作计算/调试）
			// 计算 preferred_target 到平面的有符号法向距离
			double dist_target_signed = dist_target; // 已经计算为 n.dot(preferred_target) + plane_d
			// 截断法向距离（使用起点到墙的距离 dist_R作为截断上限）
			double normal_offset = std::max(-dist_R, std::min(dist_target_signed, dist_R));
			p_virtual = p_virtual_on_wall + n * normal_offset;

			// 6. 最终计算算数中点
			baseline_pt = (p0_proj + p_virtual_on_wall) * 0.5;
		}
		else
		{
			// 异常 fallback
			baseline_pt = p0 - dist_R * n;
			p_virtual = preferred_target;
		}

		// 此时 baseline_pt 即为最终计算出的 P_aim (墙上一点)

		// Compute a horizontal tangent direction on the plane (preferred: perpendicular to plane normal)
		// Use world-up to get a stable tangent direction; fall back if nearly parallel.
		Eigen::Vector3d up(0.0, 0.0, 1.0);
		Eigen::Vector3d t = up.cross(n);
		if (t.norm() < 1e-6)
		{
			// plane normal nearly vertical; pick another axis
			up = Eigen::Vector3d(0.0, 1.0, 0.0);
			t = up.cross(n);
		}
		t.normalize();

		// sampling parameters: 6 symmetric offsets around baseline, nominal radius 0.5m
		const double nominal_r = 0.5; // meters
		const double half_width = std::max(0.0, plane_width * 0.5);

		// symmetric offsets concentrated near center
		const std::array<double,6> alphas = {{-0.6, -0.35, -0.1, 0.1, 0.35, 0.6}};

		// Use plane centroid (plane_point) as origin for safe interval and apply margin
		const double margin = 0.1; // meters distance to keep from edges
		const double L_initial = -half_width + margin;
		const double U_initial =  half_width - margin;
		double L = L_initial;
		double U = U_initial;
		if (U < L) {
			// face too narrow for margin: fall back to no-margin symmetric bounds
			L = -half_width;
			U =  half_width;
		}

		// baseline coordinate relative to centroid C in tangent axis
		const Eigen::Vector3d C = plane_point; // centroid passed in
		const double b = t.dot(baseline_pt - C);

		std::vector<Eigen::Vector3d> samples;
		samples.reserve(alphas.size());
		for (double a : alphas)
		{
			const double off_des = a * nominal_r;
			const double s_des = b + off_des; // desired coordinate in plane frame (relative to C)
			const double s = clamp(s_des, L, U);
			const double off = s - b; // actual offset to apply to baseline_pt
			Eigen::Vector3d ps = baseline_pt + off * t;
			// keep reported height equal to baseline_pt.z()
			ps.z() = baseline_pt.z();
			samples.push_back(ps);
		}

		for (size_t i = 0; i < samples.size(); ++i)
		{
			const auto &pt = samples[i];
			ROS_DEBUG("generateMixedTrajectory: sample[%zu] = %.3f %.3f %.3f", i, pt.x(), pt.y(), pt.z());
		}

		// --- 使用 bs1 模型估计最大碰撞后的速度模长 ---
		// 假设入射方向为从起点 p0 指向 baseline_pt，速度模长为 max_collision_velocity
		const Eigen::Vector3d p_coll = baseline_pt;
		Eigen::Vector3d dir_coll = p_coll - p0;
		if (dir_coll.norm() < 1e-6)
		{
			ROS_WARN("generateMixedTrajectory: degenerate collision direction (p_coll == p0)");
			return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
		}
		Eigen::Vector3d v_in = dir_coll.normalized() * max_collision_velocity;

		// 分解到法向/切向
		const Eigen::Vector3d plane_n = n; // 已使用翻转后的法向作为约定
		double v_n_algebraic = v_in.dot(plane_n);
		double approach_speed = (v_n_algebraic < 0.0) ? -v_n_algebraic : 0.0;
		Eigen::Vector3d v_t_vec = v_in - v_n_algebraic * plane_n;
		double v_t_norm = v_t_vec.norm();
		Eigen::Vector3d t_dir = Eigen::Vector3d::Zero();
		if (v_t_norm > 1e-6)
		{
			t_dir = v_t_vec / v_t_norm;
		}
		else
		{
			Eigen::Vector3d up2(0.0, 0.0, 1.0);
			if (std::abs(plane_n.dot(up2)) > 0.9)
				up2 = Eigen::Vector3d(0.0, 1.0, 0.0);
			t_dir = (up2 - up2.dot(plane_n) * plane_n).normalized();
			v_t_norm = 0.0;
		}

		// 四元数对齐与倒飞处理（同 buildFallbackCollisionPlan）
		Eigen::Quaterniond q_curr = Eigen::Quaterniond(pose.pose.orientation.w,
													   pose.pose.orientation.x,
													   pose.pose.orientation.y,
													   pose.pose.orientation.z);
		Eigen::Vector3d v_body = q_curr.inverse() * v_in;
		bool is_tail_collision = (v_body.x() < -0.1);

		Eigen::Quaterniond q_flip = Eigen::Quaterniond::Identity();
		if (is_tail_collision)
			q_flip = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

		Eigen::Vector3d train_plane_normal(-1.0, 0.0, 0.0);
		Eigen::Quaterniond q_align = Eigen::Quaterniond::Identity();
		q_align.setFromTwoVectors(plane_n, train_plane_normal);

		Eigen::Quaterniond q_input = q_align * q_curr * q_flip;

		// 组装 bs1 输入并调用 TRT
		std::vector<float> bs1_input(12, 0.0f);
		bs1_input[0] = static_cast<float>(approach_speed);
		bs1_input[1] = static_cast<float>(v_t_norm);
		bs1_input[2] = 0.0f;
		bs1_input[3] = 0.0f; bs1_input[4] = 0.0f; bs1_input[5] = 0.0f;
		bs1_input[6] = static_cast<float>(q_input.w());
		bs1_input[7] = static_cast<float>(q_input.x());
		bs1_input[8] = static_cast<float>(q_input.y());
		bs1_input[9] = static_cast<float>(q_input.z());
		const double friction = config.friction;
		const double damping_ratio = config.damping_ratio;
		bs1_input[10] = static_cast<float>(friction);
		bs1_input[11] = static_cast<float>(damping_ratio);

		std::vector<float> bs1_output;
		try
		{
			trt::TrtManager &mgr = trt::TrtManager::instance();
			if (!mgr.isReady())
			{
				ROS_WARN("generateMixedTrajectory: TrtManager not ready for bs1 prediction");
				return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
			}
			CollisionPredictor &pred = mgr.collisionBs1();
			pred.predict(bs1_input, bs1_output, 1);
		}
		catch (const std::exception &e)
		{
			ROS_WARN("generateMixedTrajectory: bs1 predict exception: %s", e.what());
			return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
		}

		if (bs1_output.size() < 2)
		{
			ROS_WARN("generateMixedTrajectory: bs1 returned insufficient outputs");
			return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
		}

		const double post_n_scalar = static_cast<double>(bs1_output[0]);
		const double post_t_scalar = static_cast<double>(bs1_output[1]);

		// 重建后碰撞速度
		const double rebound_speed = std::abs(post_n_scalar);
		Eigen::Vector3d v_n_post = rebound_speed * plane_n;
		Eigen::Vector3d v_t_post = post_t_scalar * t_dir;
		Eigen::Vector3d v_post = v_n_post + v_t_post;
		const double post_speed = v_post.norm();
		ROS_DEBUG("generateMixedTrajectory: pre_speed=%.3f post_speed=%.3f", max_collision_velocity, post_speed);

		// 下面开始构造 Student batch=72: 36 避碰 + 36 碰撞，一次性推理
		constexpr int kBatch = 72;
		constexpr int kAvoidN = 36;
		constexpr int kCollN = 36;
		static_assert(kAvoidN + kCollN == kBatch, "Batch partition mismatch");
		std::vector<float> input_raw;
		input_raw.assign(static_cast<size_t>(kBatch) * 14u, 0.0f);

		// ---- 36 条避碰样本（与原流程一致，只是 batch=36） ----
		std::vector<Eigen::Vector2d> avoid_targets_world;
		avoid_targets_world.resize(kAvoidN);
		avoid_targets_world[0] = p_virtual.head<2>();
		thread_local std::mt19937 rng(std::random_device{}());
		std::uniform_real_distribution<double> uni01(0.0, 1.0);
		std::uniform_real_distribution<double> uniAng(0.0, 2.0 * M_PI);
		const double avoid_radius_m = 3.0;
		for (int i = 1; i < kAvoidN; ++i)
		{
			const double u = uni01(rng);
			const double r = avoid_radius_m * std::sqrt(u);
			const double th = uniAng(rng);
			const Eigen::Vector2d off(r * std::cos(th), r * std::sin(th));
			avoid_targets_world[i] = p_virtual.head<2>() + off;
		}

		for (int i = 0; i < kAvoidN; ++i)
		{
			const Eigen::Vector2d geom_local = R_wb * (avoid_targets_world[i] - p0.head<2>());
			const size_t base = static_cast<size_t>(i) * 14u;
			// New Student input ordering: tm, sp, sv, sa, yv, go, gv
			// tm: task_mask (avoidance -> [0,1])
			input_raw[base + 0] = 0.0f;
			input_raw[base + 1] = 1.0f;
			// sp: start_pos (after translate+rotate) -> (0,0)
			input_raw[base + 2] = 0.0f;
			input_raw[base + 3] = 0.0f;
			// sv: start_vel
			input_raw[base + 4] = static_cast<float>(v_local.x());
			input_raw[base + 5] = static_cast<float>(v_local.y());
			// sa: start_acc
			input_raw[base + 6] = static_cast<float>(a_local.x());
			input_raw[base + 7] = static_cast<float>(a_local.y());
			// yv: yaw_vector (use real yaw) -> (cos, sin)
			input_raw[base + 8] = static_cast<float>(yaw_fwd_world.x());
			input_raw[base + 9] = static_cast<float>(yaw_fwd_world.y());
			// go: geometry_offset
			input_raw[base + 10] = static_cast<float>(geom_local.x());
			input_raw[base + 11] = static_cast<float>(geom_local.y());
			// gv: guide_vector (fixed for avoidance)
			input_raw[base + 12] = 1.0f;
			input_raw[base + 13] = 0.0f;
		}

		// ---- 36 条碰撞样本：8 个碰撞点 × 3 个速度系数 × 3 个 yaw ----
		// yaw 生成规则：若碰撞点在机体系左侧(y>0)，则 {0, rand[0,20], rand[0,20]}
		// 若在右侧(y<0)，则 {0, rand[-20,0], rand[-20,0]}
		// Attention: 新模型中的yaw是cos先后sin，与旧模型相反
		std::uniform_real_distribution<double> uniLeft(0.0, 20.0);
		std::uniform_real_distribution<double> uniRight(-20.0, 0.0);
		const std::array<double,2> speed_scales = {{1.0, 0.5}};
		const double max_post_speed = post_speed;
		int out_i = kAvoidN;
		for (int si = 0; si < static_cast<int>(samples.size()); ++si)
		{
			const Eigen::Vector3d coll_pt = samples[si];
			const Eigen::Vector2d coll_local = R_wb * (coll_pt.head<2>() - p0.head<2>());
			const bool is_left = (coll_local.y() > 0.0);
			double yaw_deg_1 = is_left ? uniLeft(rng) : uniRight(rng);
			double yaw_deg_2 = is_left ? uniLeft(rng) : uniRight(rng);
			const std::array<double,3> yaws_deg = {{0.0, yaw_deg_1, yaw_deg_2}};

			// guide direction: from collision point to p_virtual, expressed in body frame
			Eigen::Vector2d post_dir_world = p_virtual.head<2>() - coll_pt.head<2>();
			Eigen::Vector2d post_dir_body = R_wb * post_dir_world;
			if (!post_dir_body.allFinite() || post_dir_body.norm() < 1e-6)
			{
				ROS_WARN("generateMixedTrajectory: degenerate post-collision direction at sample %d", si);
				return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
			}
			post_dir_body.normalize();

			for (double yaw_deg : yaws_deg)
			{
				const double yaw_rad = yaw_deg * M_PI / 180.0;
				// Model expects yaw input as (cos, sin) in body frame (new ordering)
				const Eigen::Vector2d yaw_vec_body(std::cos(yaw_rad), std::sin(yaw_rad));
				for (double sc : speed_scales)
				{
					if (out_i >= kBatch)
						break;
					const size_t base = static_cast<size_t>(out_i) * 14u;
					// New Student input ordering: tm, sp, sv, sa, yv, go, gv
					// tm: task_mask (collision -> [1,0])
					input_raw[base + 0] = 1.0f;
					input_raw[base + 1] = 0.0f;
					// sp: start_pos (after translate+rotate) -> (0,0)
					input_raw[base + 2] = 0.0f;
					input_raw[base + 3] = 0.0f;
					// sv: start_vel (use speed hack for x)
					input_raw[base + 4] = static_cast<float>(v_local.x());
					input_raw[base + 5] = static_cast<float>(v_local.y());
					// sa: start_acc
					input_raw[base + 6] = static_cast<float>(a_local.x());
					input_raw[base + 7] = static_cast<float>(a_local.y());
					// yv: yaw_vector (selected based on left/right rule), body-frame (cos, sin)
					input_raw[base + 8] = static_cast<float>(yaw_vec_body.x());
					input_raw[base + 9] = static_cast<float>(yaw_vec_body.y());
					// go: geometry_offset: collision point position in body frame
					input_raw[base + 10] = static_cast<float>(coll_local.x());
					input_raw[base + 11] = static_cast<float>(coll_local.y());
					// gv: guide_vector: body-frame post-collision velocity
					const Eigen::Vector2d guide = post_dir_body * (max_post_speed * sc);
					input_raw[base + 12] = static_cast<float>(guide.x());
					input_raw[base + 13] = static_cast<float>(guide.y());
					out_i++;
				}
			}
		}
		if (out_i != kBatch)
		{
			ROS_WARN("generateMixedTrajectory: collision batch filled %d/%d", out_i, kBatch);
			return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
		}

		// ---- 单次 Student 推理（必须可用；否则直接返回空轨迹） ----
		StudentPredictor::RemappedBatch remapped;
		try
		{
			// Dump 36 collision batch inputs for offline debugging
			{
				const std::string fname = resolveDataCsvPathIntention(std::string("student_collision_inputs_.csv"));
				std::ofstream fout(fname, std::ios::out);
				if (fout.is_open()) {
					fout << std::setprecision(6) << std::fixed;
					// header: sample_id, then 14 input fields
					fout << "sample_id";
					for (int j = 0; j < 14; ++j) fout << ",in" << j;
					fout << "\n";
					for (int bi = 0; bi < kBatch; ++bi) {
						size_t base = static_cast<size_t>(bi) * 14u;
						fout << bi;
						for (int j = 0; j < 14; ++j) {
							fout << "," << static_cast<double>(input_raw[base + j]);
						}
						fout << "\n";
					}
					fout.close();
					ROS_INFO("Saved student collision batch inputs to %s", fname.c_str());
				} else {
					ROS_WARN("Failed to open %s for writing student inputs", fname.c_str());
				}
			}
			trt::TrtManager &mgr = trt::TrtManager::instance();
			if (!mgr.isReady())
			{
				ROS_WARN("generateMixedTrajectory: TrtManager not ready - student unavailable");
				return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
			}
			StudentPredictor &student = mgr.student();
			student.predict_remap(input_raw, remapped, kBatch);
			if (remapped.control_points_padded.size() < static_cast<size_t>(kBatch) * 21u * 2u ||
				remapped.lengths.size() != static_cast<size_t>(kBatch) ||
				remapped.v_pre.size() < static_cast<size_t>(kBatch) * 2u ||
				remapped.t_post.size() < static_cast<size_t>(kBatch))
			{
				ROS_WARN("generateMixedTrajectory: student remap returned insufficient outputs (cp=%zu len=%zu vpre=%zu)",
						 remapped.control_points_padded.size(), remapped.lengths.size(), remapped.v_pre.size());
				return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
			}

			// Dump student remapped outputs for offline debugging
			{
				const std::string fname_out = resolveDataCsvPathIntention(std::string("student_collision_outputs_.csv"));
				std::ofstream fout_out(fname_out, std::ios::out);
				if (fout_out.is_open()) {
					fout_out << std::setprecision(6) << std::fixed;
					// header
					fout_out << "sample_id,length,t_post,vpre_x,vpre_y";
					for (int i = 0; i < 21; ++i) {
						fout_out << ",cp_x" << i << ",cp_y" << i;
					}
					fout_out << "\n";
					for (int bi = 0; bi < kBatch; ++bi) {
						size_t base_cp = static_cast<size_t>(bi) * 21u * 2u;
						fout_out << bi << "," << remapped.lengths[static_cast<size_t>(bi)] << "," << remapped.t_post[static_cast<size_t>(bi)];
						// v_pre
						size_t base_vpre = static_cast<size_t>(bi) * 2u;
						fout_out << "," << remapped.v_pre[base_vpre + 0] << "," << remapped.v_pre[base_vpre + 1];
						// control points (21 pairs)
						for (int ci = 0; ci < 21; ++ci) {
							fout_out << "," << remapped.control_points_padded[base_cp + static_cast<size_t>(ci * 2 + 0)]
										<< "," << remapped.control_points_padded[base_cp + static_cast<size_t>(ci * 2 + 1)];
						}
						fout_out << "\n";
					}
					fout_out.close();
					ROS_INFO("Saved student remapped outputs to %s", fname_out.c_str());
				} else {
					ROS_WARN("Failed to open %s for writing student outputs", fname_out.c_str());
				}
			}
		}
		catch (const std::exception &e)
		{
			ROS_WARN("generateMixedTrajectory: student predict exception: %s", e.what());
			return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
		}

		// ---- 36 条碰撞轨迹：直接从 unified remap 提取碰撞前速度 v_pre（机体系 -> 世界系） ----
		// 碰撞后段时间 t_post 由 remap_batch 内部估算并输出；这里直接使用：
		//   T_pre = 4.0 - t_post
		std::array<Eigen::Vector2d, kCollN> v_pre_body_xy;
		std::array<Eigen::Vector2d, kCollN> v_pre_world_xy;
		std::array<double, kCollN> pre_ratio;
		std::array<double, kCollN> T_post_s;
		v_pre_body_xy.fill(Eigen::Vector2d::Zero());
		v_pre_world_xy.fill(Eigen::Vector2d::Zero());
		pre_ratio.fill(1.0);
		T_post_s.fill(0.0);
		{
			constexpr double kTotalT = 4.0;
			for (int ci = 0; ci < kCollN; ++ci)
			{
				const int bi = kAvoidN + ci;

				// v_pre provided by remap (physical units, body frame)
				const Eigen::Vector2d v_pre_b(remapped.v_pre[static_cast<size_t>(bi) * 2u + 0u],
								 remapped.v_pre[static_cast<size_t>(bi) * 2u + 1u]);
				v_pre_body_xy[ci] = v_pre_b;
				v_pre_world_xy[ci] = R_bw * v_pre_b;

				const double t_post = std::max(0.0, static_cast<double>(remapped.t_post[static_cast<size_t>(bi)]));
				double T_pre = kTotalT - t_post;
				if (!std::isfinite(T_pre) || T_pre <= 1e-3)
				{
					ROS_WARN("generateMixedTrajectory: degenerate T_pre=%.6f (ci=%d, t_post=%.6f)", T_pre, ci, t_post);
					T_pre = std::max(1e-3, kTotalT);
				}
				pre_ratio[ci] = clamp(T_pre / kTotalT, 0.0, 1.0);
				T_post_s[ci] = std::min(kTotalT, std::max(0.0, t_post));
			}
		}

		// ---- 使用 bs36 重新估计 36 条碰撞样本的碰撞后速度，并按直线段重建碰撞后轨迹 ----
		// 输入处理、姿态对齐与倒飞处理均与 bs1 一致。
		std::array<Eigen::Vector3d, kCollN> t_dir_world;
		std::array<Eigen::Vector3d, kCollN> v_post_world;
		t_dir_world.fill(Eigen::Vector3d::Zero());
		v_post_world.fill(Eigen::Vector3d::Zero());

		bool bs36_ok = false;
		std::vector<float> bs36_in(static_cast<size_t>(kCollN) * 12u, 0.0f);
		std::vector<float> bs36_out;
		{
			// plane_n uses flipped normal convention
			const Eigen::Vector3d plane_n = n;
			Eigen::Quaterniond q_curr = Eigen::Quaterniond(pose.pose.orientation.w,
											 pose.pose.orientation.x,
											 pose.pose.orientation.y,
											 pose.pose.orientation.z);
			Eigen::Vector3d train_plane_normal(-1.0, 0.0, 0.0);
			Eigen::Quaterniond q_align = Eigen::Quaterniond::Identity();
			q_align.setFromTwoVectors(plane_n, train_plane_normal);

			for (int ci = 0; ci < kCollN; ++ci)
			{
				const Eigen::Vector3d v_in(v_pre_world_xy[ci].x(), v_pre_world_xy[ci].y(), 0.0);
				double v_n_algebraic = v_in.dot(plane_n);
				double approach_speed = (v_n_algebraic < 0.0) ? -v_n_algebraic : 0.0;
				Eigen::Vector3d v_t_vec = v_in - v_n_algebraic * plane_n;
				double v_t_norm = v_t_vec.norm();
				Eigen::Vector3d t_dir = Eigen::Vector3d::Zero();
				if (v_t_norm > 1e-6)
				{
					t_dir = v_t_vec / v_t_norm;
				}
				else
				{
					Eigen::Vector3d up2(0.0, 0.0, 1.0);
					if (std::abs(plane_n.dot(up2)) > 0.9)
						up2 = Eigen::Vector3d(0.0, 1.0, 0.0);
					t_dir = (up2 - up2.dot(plane_n) * plane_n).normalized();
					v_t_norm = 0.0;
				}
				t_dir_world[ci] = t_dir;

				Eigen::Vector3d v_body = q_curr.inverse() * v_in;
				bool is_tail_collision = (v_body.x() < -0.1);
				Eigen::Quaterniond q_flip = Eigen::Quaterniond::Identity();
				if (is_tail_collision)
					q_flip = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
				// reconstruct body-yaw from earlier student inputs (stored as (cos,sin) in input_raw)
				const int bi_input = kAvoidN + ci;
				const size_t base_input = static_cast<size_t>(bi_input) * 14u;
				double cos_yaw = 1.0, sin_yaw = 0.0;
				if (base_input + 9u < input_raw.size()) {
					cos_yaw = static_cast<double>(input_raw[base_input + 8]);
					sin_yaw = static_cast<double>(input_raw[base_input + 9]);
				}
				const double yaw_body = std::atan2(sin_yaw, cos_yaw);
				const Eigen::Quaterniond q_yaw = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()));
				const Eigen::Quaterniond q_used = q_curr * q_yaw;
				Eigen::Quaterniond q_input = q_align * q_used * q_flip;

				const size_t base = static_cast<size_t>(ci) * 12u;
				bs36_in[base + 0] = static_cast<float>(approach_speed);
				bs36_in[base + 1] = static_cast<float>(v_t_norm);
				bs36_in[base + 2] = 0.0f;
				bs36_in[base + 3] = 0.0f;
				bs36_in[base + 4] = 0.0f;
				bs36_in[base + 5] = 0.0f;
				bs36_in[base + 6] = static_cast<float>(q_input.w());
				bs36_in[base + 7] = static_cast<float>(q_input.x());
				bs36_in[base + 8] = static_cast<float>(q_input.y());
				bs36_in[base + 9] = static_cast<float>(q_input.z());
				bs36_in[base + 10] = static_cast<float>(config.friction);
				bs36_in[base + 11] = static_cast<float>(config.damping_ratio);
			}

			try
			{
				trt::TrtManager &mgr = trt::TrtManager::instance();
				if (mgr.isReady())
				{
					CollisionPredictor &pred = mgr.collisionBs36();
					pred.predict(bs36_in, bs36_out, kCollN);
					// each sample returns 6 values: post_n, post_t, vz, wx, wy, wz
					bs36_ok = (bs36_out.size() >= static_cast<size_t>(kCollN) * 6u);
				}
				else
				{
					ROS_WARN("generateMixedTrajectory: TrtManager not ready - bs36 unavailable, fallback to guide_vector");
					bs36_ok = false;
				}
			}
			catch (const std::exception &e)
			{
				ROS_WARN("generateMixedTrajectory: bs36 predict exception: %s", e.what());
				bs36_ok = false;
			}
		}

		{
			const Eigen::Vector3d plane_n = n;
			for (int ci = 0; ci < kCollN; ++ci)
			{
				if (bs36_ok)
				{
					// bs36_out layout: for each sample 6 values: [post_n, post_t, vz, wx, wy, wz]
					const double post_n_scalar = static_cast<double>(bs36_out[static_cast<size_t>(ci) * 6u + 0u]);
					const double post_t_scalar = static_cast<double>(bs36_out[static_cast<size_t>(ci) * 6u + 1u]);
					const double rebound_speed = std::abs(post_n_scalar);
					Eigen::Vector3d v_n_post = rebound_speed * plane_n;
					Eigen::Vector3d v_t_post = post_t_scalar * t_dir_world[ci];
					v_post_world[ci] = v_n_post + v_t_post;
				}
				else
				{
					// fallback: use student input guide_vector as post-collision velocity (body -> world)
					const int bi = kAvoidN + ci;
					const size_t base_in = static_cast<size_t>(bi) * 14u;
					const Eigen::Vector2d v_post_body(input_raw[base_in + 6], input_raw[base_in + 7]);
					const Eigen::Vector2d v_post_w_xy = R_bw * v_post_body;
					v_post_world[ci] = Eigen::Vector3d(v_post_w_xy.x(), v_post_w_xy.y(), 0.0);
				}
			}
		}

		// ---- 生成 72 条轨迹（B-spline 控制点）：避碰18点/碰撞21点 ----
		std::vector<quadrotor_msgs::TrajectoryPlan> plans;
		plans.resize(kBatch);

		// preferred direction in XY used as fallback when post-collision direction is invalid
		Eigen::Vector2d pref_dir(preferred_dir_unit.x(), preferred_dir_unit.y());
		if (!pref_dir.allFinite() || pref_dir.norm() < 1e-6) pref_dir = Eigen::Vector2d(1.0, 0.0);
		else pref_dir.normalize();

		for (int bi = 0; bi < kBatch; ++bi)
		{
			const bool is_collision = (bi >= kAvoidN);
			const int len = (bi >= 0 && bi < kBatch) ? remapped.lengths[bi] : 0;
			if ((!is_collision && len != 18) || (is_collision && len != 21))
			{
				// unexpected remap length: fall back to a safe straight-line avoidance plan
				const Eigen::Vector3d p1(avoid_targets_world[0].x(), avoid_targets_world[0].y(), preferred_target.z());
				plans[bi] = buildStraightLineBsplineCtrlPlan(p0, p1, preferred_dir_unit, quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE);
				continue;
			}

			std::vector<Eigen::Vector2d> ctrl_xy_world;
			ctrl_xy_world.resize(static_cast<size_t>(len));
			for (int j = 0; j < len; ++j)
			{
				const size_t fidx = (static_cast<size_t>(bi) * 21u + static_cast<size_t>(j)) * 2u;
				const Eigen::Vector2d cp_local(remapped.control_points_padded[fidx + 0u], remapped.control_points_padded[fidx + 1u]);
				ctrl_xy_world[static_cast<size_t>(j)] = p0.head<2>() + R_bw * cp_local;
			}
			// enforce start position
			ctrl_xy_world.front() = p0.head<2>();

			const int mode = is_collision ? quadrotor_msgs::TrajectoryPlan::MODE_COLLISION
							 : quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE;

			if (!is_collision)
			{
				// avoidance: enforce avoidance end target and use full control points
				ctrl_xy_world.back() = avoid_targets_world[bi];
				Eigen::Vector3d p_end = Eigen::Vector3d::Zero();
				p_end.x() = ctrl_xy_world.back().x();
				p_end.y() = ctrl_xy_world.back().y();
				p_end.z() = preferred_target.z();
				fillBsplinePlanFromWorldCtrlPointsVar(plans[bi], p0, p_end, preferred_dir_unit, ctrl_xy_world, SAMPLE_FORWARD_BSPLINE_DEGREE, mode);
			}
			else
			{
				// collision: only keep pre-collision control points (first 16)
				constexpr int CtrlPre = 16;
				std::vector<Eigen::Vector2d> ctrl_pre;
				ctrl_pre.resize(static_cast<size_t>(CtrlPre));
				for (int j = 0; j < CtrlPre && j < static_cast<int>(ctrl_xy_world.size()); ++j)
					ctrl_pre[static_cast<size_t>(j)] = ctrl_xy_world[static_cast<size_t>(j)];
				// enforce start
				ctrl_pre.front() = p0.head<2>();
				Eigen::Vector3d p_end = Eigen::Vector3d::Zero();
				p_end.x() = ctrl_pre.back().x();
				p_end.y() = ctrl_pre.back().y();
				p_end.z() = preferred_target.z();
				fillBsplinePlanFromWorldCtrlPointsVar(plans[bi], p0, p_end, preferred_dir_unit, ctrl_pre, SAMPLE_FORWARD_BSPLINE_DEGREE, mode);
			}
		}

		// ---- 混合筛选：36 避碰 + 36 碰撞，取总代价最小 ----
		// Read kinematic limits from YAML-loaded ROS params
		double max_vel = 4.0, max_acc = 4.0;
		readMaxVelAccFromRosParams(max_vel, max_acc);

		// Pre-build corridor group matrices for vectorized violation evaluation
		std::vector<Eigen::MatrixXd> group_normals;
		std::vector<Eigen::VectorXd> group_ds;
		buildCorridorGroupMatrices(plane_candidates, group_normals, group_ds);

		constexpr int M = 42; // samples along the spline
		const double total_T = 4.0;
		const double dt = total_T / static_cast<double>(M - 1);

		int best_i = 0;

		// Vectorized scoring buffers (direction/end/total) across kBatch
		Eigen::Array<bool, Eigen::Dynamic, 1> valid_plan(kBatch);
		valid_plan.setConstant(false);
		Eigen::ArrayXd coll_norm_vec = Eigen::ArrayXd::Zero(kBatch);
		Eigen::ArrayXd kin_cost_vec = Eigen::ArrayXd::Zero(kBatch);
		Eigen::ArrayXd collision_mask = Eigen::ArrayXd::Zero(kBatch); // 1.0 for collision plans, else 0.0
		Eigen::Matrix<double, 2, Eigen::Dynamic> end_xy_mat(2, kBatch);
		Eigen::Matrix<double, 2, Eigen::Dynamic> back_xy_mat(2, kBatch);
		Eigen::Matrix<double, 2, Eigen::Dynamic> tan_coll_mat(2, kBatch);
		end_xy_mat.setZero();
		back_xy_mat.setZero();
		tan_coll_mat.setZero();

		// Cache sampled points for CSV debugging (must live outside inner scopes).
		constexpr int Mpre_dbg = 28;
		constexpr int Mpost_dbg = M - Mpre_dbg;
		static_assert(Mpost_dbg > 1, "Need at least 2 post samples");
		Eigen::MatrixXd X_avoid(M, kAvoidN);
		Eigen::MatrixXd Y_avoid(M, kAvoidN);
		Eigen::MatrixXd Xpre_dbg(Mpre_dbg, kCollN);
		Eigen::MatrixXd Ypre_dbg(Mpre_dbg, kCollN);
		Eigen::MatrixXd Xpost_dbg(Mpost_dbg, kCollN);
		Eigen::MatrixXd Ypost_dbg(Mpost_dbg, kCollN);
		X_avoid.setZero();
		Y_avoid.setZero();
		Xpre_dbg.setZero();
		Ypre_dbg.setZero();
		Xpost_dbg.setZero();
		Ypost_dbg.setZero();

		// avoidance direction uses P(T) - P(T-1.3s)
		const double lookback_s = 1.3;
		const int lookback_steps = static_cast<int>(std::lround(lookback_s / dt));
		const int k0 = clamp((M - 1) - lookback_steps, 0, M - 1);

		// ---- Mixed: batch sample points for collision + kinematics costs ----
		// - avoidance: 18 ctrl points, degree 5, T=4, 42 samples (same as generateAvoidanceTrajectory)
		// - collision: pre segment uses 16 ctrl points, degree 5, T_pre from remap;
		//             post segment sampled on a straight line; post does NOT enter kinematics.
		//             collision point "before/after" samples are excluded from collision cost.
		{
			constexpr int Mpre = 28;
			constexpr int Mpost = M - Mpre;
			constexpr int CtrlA = 18;
			constexpr int CtrlPre = 16;
			static_assert(Mpost > 1, "Need at least 2 post samples");

			const double z = p0.z();
			// ---- Avoidance batch (36) ----
			{
				const auto &avoid_mats = getAvoidanceBsplineMatrices();
				const Eigen::MatrixXd &Bt_a = avoid_mats.Bt_pos;
				const Eigen::MatrixXd &Bt_a_vel = avoid_mats.Bt_vel;
				const Eigen::MatrixXd &Bt_a_acc = avoid_mats.Bt_acc;

				Eigen::MatrixXd Cx(CtrlA, kAvoidN);
				Eigen::MatrixXd Cy(CtrlA, kAvoidN);
				Cx.setZero();
				Cy.setZero();
				for (int ai = 0; ai < kAvoidN; ++ai)
				{
					const auto &plan = plans[ai];
					if (static_cast<int>(plan.waypoints.size()) < CtrlA)
						continue;
					valid_plan(ai) = true;
					collision_mask(ai) = 0.0;
					for (int j = 0; j < CtrlA; ++j)
					{
						Cx(j, ai) = plan.waypoints[j].x;
						Cy(j, ai) = plan.waypoints[j].y;
					}
				}
				const Eigen::MatrixXd X = Bt_a * Cx; // (M x kAvoidN)
				const Eigen::MatrixXd Y = Bt_a * Cy; // (M x kAvoidN)
				X_avoid = X;
				Y_avoid = Y;

				end_xy_mat.block(0, 0, 1, kAvoidN) = X.row(M - 1);
				end_xy_mat.block(1, 0, 1, kAvoidN) = Y.row(M - 1);
				back_xy_mat.block(0, 0, 1, kAvoidN) = X.row(k0);
				back_xy_mat.block(1, 0, 1, kAvoidN) = Y.row(k0);

				// collision cost
				if (!group_normals.empty())
				{
					Eigen::Matrix3Xd P_big(3, M * kAvoidN);
					P_big.setZero();
					for (int ai = 0; ai < kAvoidN; ++ai)
					{
						P_big.block(0, ai * M, 1, M) = X.col(ai).transpose().eval();
						P_big.block(1, ai * M, 1, M) = Y.col(ai).transpose().eval();
						P_big.block(2, ai * M, 1, M).setConstant(z);
					}
					coll_norm_vec.head(kAvoidN) = corridorViolationCostBatchVectorized(P_big, M, kAvoidN, group_normals, group_ds);
				}

				// kinematic cost (T=4) - Analytic Derivatives
				const double spans_a = static_cast<double>(CtrlA - SAMPLE_FORWARD_BSPLINE_DEGREE);
				const double scale_v_a = spans_a / total_T;
				const double scale_a_a = scale_v_a * scale_v_a;

				const Eigen::MatrixXd V_u_x = Bt_a_vel * Cx;
				const Eigen::MatrixXd V_u_y = Bt_a_vel * Cy;
				const Eigen::MatrixXd A_u_x = Bt_a_acc * Cx;
				const Eigen::MatrixXd A_u_y = Bt_a_acc * Cy;

				const Eigen::MatrixXd Vx = V_u_x * scale_v_a;
				const Eigen::MatrixXd Vy = V_u_y * scale_v_a;
				const Eigen::MatrixXd Ax = A_u_x * scale_a_a;
				const Eigen::MatrixXd Ay = A_u_y * scale_a_a;

				const Eigen::ArrayXd max_abs_vx = Vx.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
				const Eigen::ArrayXd max_abs_vy = Vy.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
				const Eigen::ArrayXd max_abs_ax = Ax.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
				const Eigen::ArrayXd max_abs_ay = Ay.cwiseAbs().colwise().maxCoeff().transpose().eval().array();

				Eigen::ArrayXd v_cost = Eigen::ArrayXd::Zero(kAvoidN);
				v_cost += ((max_abs_vx - max_vel) / max_vel).max(0.0);
				v_cost += ((max_abs_vy - max_vel) / max_vel).max(0.0);
				Eigen::ArrayXd a_cost = Eigen::ArrayXd::Zero(kAvoidN);
				a_cost += ((max_abs_ax - max_acc) / max_acc).max(0.0);
				a_cost += ((max_abs_ay - max_acc) / max_acc).max(0.0);
				kin_cost_vec.head(kAvoidN) = v_cost + a_cost;
			}

			// ---- Collision batch (36): pre (B-spline) + post (line) for collision cost; pre only for kinematics ----
			{
				const auto &pre_mats = getCollisionPreBsplineMatrices();
				const Eigen::MatrixXd &Bt_pre = pre_mats.Bt_pos;
				const Eigen::MatrixXd &Bt_pre_vel = pre_mats.Bt_vel;
				const Eigen::MatrixXd &Bt_pre_acc = pre_mats.Bt_acc;

				Eigen::MatrixXd Cx_pre(CtrlPre, kCollN);
				Eigen::MatrixXd Cy_pre(CtrlPre, kCollN);
				Cx_pre.setZero();
				Cy_pre.setZero();

				Eigen::ArrayXd T_pre = Eigen::ArrayXd::Constant(kCollN, 1e-3);
				Eigen::ArrayXd T_post = Eigen::ArrayXd::Zero(kCollN);
				Eigen::ArrayXd v_post_x = Eigen::ArrayXd::Zero(kCollN);
				Eigen::ArrayXd v_post_y = Eigen::ArrayXd::Zero(kCollN);

				for (int ci = 0; ci < kCollN; ++ci)
				{
					const int bi = kAvoidN + ci;
					const auto &plan = plans[bi];
					if (static_cast<int>(plan.waypoints.size()) < 16)
						continue;
					valid_plan(bi) = true;
					collision_mask(bi) = 1.0;

					for (int j = 0; j < CtrlPre; ++j)
					{
						Cx_pre(j, ci) = plan.waypoints[j].x;
						Cy_pre(j, ci) = plan.waypoints[j].y;
					}

					// collision end point used for endpoint cost
					end_xy_mat(0, bi) = plan.waypoints.back().x;
					end_xy_mat(1, bi) = plan.waypoints.back().y;

					// durations and post velocity for sampling
					const double r = clamp(pre_ratio[ci], 0.0, 1.0);
					T_pre(ci) = std::max(1e-3, r * total_T);
					T_post(ci) = std::max(0.0, static_cast<double>(T_post_s[ci]));
					v_post_x(ci) = v_post_world[ci].x();
					v_post_y(ci) = v_post_world[ci].y();
				}

				const Eigen::MatrixXd Xpre = Bt_pre * Cx_pre; // (Mpre x kCollN)
				const Eigen::MatrixXd Ypre = Bt_pre * Cy_pre; // (Mpre x kCollN)
				Xpre_dbg = Xpre;
				Ypre_dbg = Ypre;

				// Build post samples using min-jerk quintic (start at p_coll with v_post, end with v=0)
				Eigen::MatrixXd Xpost(Mpost, kCollN);
				Eigen::MatrixXd Ypost(Mpost, kCollN);
				Xpost.setZero();
				Ypost.setZero();
				for (int ci = 0; ci < kCollN; ++ci)
				{
					// collision point from pre samples (last pre sample)
					const double x_coll = Xpre.row(Mpre - 1)(0, ci);
					const double y_coll = Ypre.row(Mpre - 1)(0, ci);
					const double Tpost = std::max(0.0, static_cast<double>(T_post(ci)));
					const double vx_post = v_post_x(ci);
					const double vy_post = v_post_y(ci);

					if (Tpost <= 1e-6 || !std::isfinite(Tpost))
					{
						// degenerate: fill with collision point
						for (int m = 0; m < Mpost; ++m)
						{
							Xpost(m, ci) = x_coll;
							Ypost(m, ci) = y_coll;
						}
						continue;
					}

					// Use buildPostCollisionMinJerkDenseSamples to build a dense min-jerk curve,
					// then resample to exactly Mpost points excluding the start (collision point).
					const double x_end = x_coll + vx_post * Tpost;  // *0.5 removed for consistency
					const double y_end = y_coll + vy_post * Tpost; // *0.5 removed for consistency

					const int bi = kAvoidN + ci;

					// prepare 3D vectors for dense builder
					Eigen::Vector3d p_coll3d(x_coll, y_coll, z);
					Eigen::Vector3d v0(vx_post, vy_post, 0.0);
					std::vector<Eigen::Vector3d> dense_pts;
					std::vector<double> dense_t;
					buildPostCollisionMinJerkDenseSamples(p_coll3d, v0, Tpost, dense_pts, dense_t);

					// compute total arc length
					double total_len = 0.0;
					for (size_t ii = 0; ii + 1 < dense_pts.size(); ++ii)
						total_len += (dense_pts[ii + 1] - dense_pts[ii]).norm();

					if (total_len <= 1e-8 || dense_pts.size() < 2)
					{
						// degenerate: fill with collision point
						for (int m = 0; m < Mpost; ++m)
						{
							Xpost(m, ci) = x_coll;
							Ypost(m, ci) = y_coll;
						}
						// set tangent to zero-length
						if (bi >= 0 && bi < kBatch) {
							tan_coll_mat(0, bi) = 0.0;
							tan_coll_mat(1, bi) = 0.0;
						}
						continue;
					}

					// target spacing so that resampling yields ~Mpost+1 points (including start)
					const double spacing = total_len / static_cast<double>(Mpost);
					std::vector<Eigen::Vector3d> post_pts;
					std::vector<double> post_ts;
					resampleMethod2ArcLength(dense_pts, dense_t, spacing, post_pts, post_ts);

					// If resample produced at least Mpost+1 points (start included), take indices 1..Mpost
					if (post_pts.size() >= static_cast<size_t>(Mpost + 1))
					{
						for (int m = 0; m < Mpost; ++m)
						{
							const auto &pp = post_pts[m + 1]; // skip start
							Xpost(m, ci) = pp.x();
							Ypost(m, ci) = pp.y();
						}
						// set tangent from collision point to final post sample
						if (bi >= 0 && bi < kBatch) {
							const auto &pend = post_pts[post_pts.size() - 1];
							tan_coll_mat(0, bi) = pend.x() - x_coll;
							tan_coll_mat(1, bi) = pend.y() - y_coll;
						}
					}
					else
					{
						// fallback: sample quintic at Mpost times excluding t=0
						const Quintic1D qx = fitQuinticBoundary1D(x_coll, vx_post, 0.0, x_end, 0.0, 0.0, Tpost);
						const Quintic1D qy = fitQuinticBoundary1D(y_coll, vy_post, 0.0, y_end, 0.0, 0.0, Tpost);
						for (int m = 0; m < Mpost; ++m)
						{
							const double t = (static_cast<double>(m + 1) / static_cast<double>(Mpost)) * Tpost; // exclude t=0
							Xpost(m, ci) = evalQuintic1D(qx, t);
							Ypost(m, ci) = evalQuintic1D(qy, t);
						}
						if (bi >= 0 && bi < kBatch) {
							tan_coll_mat(0, bi) = x_end - x_coll;
							tan_coll_mat(1, bi) = y_end - y_coll;
						}
					}
				}
				Xpost_dbg = Xpost;
				Ypost_dbg = Ypost;

				// collision cost uses concatenated samples; exclude collision point "before/after"
				if (!group_normals.empty())
				{
					Eigen::Matrix3Xd P_big(3, M * kCollN);
					P_big.setZero();
					for (int ci = 0; ci < kCollN; ++ci)
					{
						P_big.block(0, ci * M, 1, Mpre) = Xpre.col(ci).transpose().eval();
						P_big.block(1, ci * M, 1, Mpre) = Ypre.col(ci).transpose().eval();
						P_big.block(0, ci * M + Mpre, 1, Mpost) = Xpost.col(ci).transpose().eval();
						P_big.block(1, ci * M + Mpre, 1, Mpost) = Ypost.col(ci).transpose().eval();
						P_big.block(2, ci * M, 1, M).setConstant(z);
					}
					Eigen::MatrixXd mask = Eigen::MatrixXd::Ones(M, kCollN);
					mask.row(Mpre - 1).setZero();
					mask.row(Mpre).setZero();
					coll_norm_vec.tail(kCollN) = corridorViolationCostBatchVectorizedMasked(P_big, M, kCollN, group_normals, group_ds, mask);
				}

				// kinematic cost: Analytic derivatives (Batch)
				// u in [0, 11] for N=16, p=5. du/dt = 11.0 / T_pre
				const double spans_pre = static_cast<double>(CtrlPre - SAMPLE_FORWARD_BSPLINE_DEGREE);
				// Clamp T_pre to avoid singularity (e.g. if T_pre < 0.2s, acc implies enormous force)
				const Eigen::ArrayXd T_pre_safe = T_pre.max(0.2); 
				const Eigen::ArrayXd scale_v = spans_pre / T_pre_safe;
				const Eigen::ArrayXd scale_a = scale_v.square();

				// Evaluate parameter derivatives (M x Batch) via precomputed basis matrices
				const Eigen::MatrixXd V_u_x = Bt_pre_vel * Cx_pre;
				const Eigen::MatrixXd V_u_y = Bt_pre_vel * Cy_pre;
				const Eigen::MatrixXd A_u_x = Bt_pre_acc * Cx_pre;
				const Eigen::MatrixXd A_u_y = Bt_pre_acc * Cy_pre;

				// Scale to time derivatives: v = (dC/du)*(du/dt), a = (d2C/du2)*(du/dt)^2
				const Eigen::MatrixXd Vx = (V_u_x.array().rowwise() * scale_v.transpose()).matrix();
				const Eigen::MatrixXd Vy = (V_u_y.array().rowwise() * scale_v.transpose()).matrix();
				const Eigen::MatrixXd Ax = (A_u_x.array().rowwise() * scale_a.transpose()).matrix();
				const Eigen::MatrixXd Ay = (A_u_y.array().rowwise() * scale_a.transpose()).matrix();

				const Eigen::ArrayXd max_abs_vx = Vx.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
				const Eigen::ArrayXd max_abs_vy = Vy.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
				const Eigen::ArrayXd max_abs_ax = Ax.cwiseAbs().colwise().maxCoeff().transpose().eval().array();
				const Eigen::ArrayXd max_abs_ay = Ay.cwiseAbs().colwise().maxCoeff().transpose().eval().array();

				Eigen::ArrayXd v_cost = Eigen::ArrayXd::Zero(kCollN);
				v_cost += ((max_abs_vx - max_vel) / max_vel).max(0.0);
				v_cost += ((max_abs_vy - max_vel) / max_vel).max(0.0);
				Eigen::ArrayXd a_cost = Eigen::ArrayXd::Zero(kCollN);
				a_cost += ((max_abs_ax - max_acc) / max_acc).max(0.0);
				a_cost += ((max_abs_ay - max_acc) / max_acc).max(0.0);
				kin_cost_vec.tail(kCollN) = v_cost + a_cost;
			}
		}

		// Vectorized direction/end costs (Scheme A for avoidance, Scheme B for collision)
		if (kBatch > 0)
		{
			const Eigen::ArrayXd mask_c = collision_mask;
			const Eigen::ArrayXd mask_a = (1.0 - mask_c);

			// direction vectors
			const Eigen::ArrayXd tan_ax = (end_xy_mat.row(0).array() - back_xy_mat.row(0).array());
			const Eigen::ArrayXd tan_ay = (end_xy_mat.row(1).array() - back_xy_mat.row(1).array());
			const Eigen::ArrayXd tan_cx = tan_coll_mat.row(0).array();
			const Eigen::ArrayXd tan_cy = tan_coll_mat.row(1).array();
			const Eigen::ArrayXd tan_x = tan_ax * mask_a + tan_cx * mask_c;
			const Eigen::ArrayXd tan_y = tan_ay * mask_a + tan_cy * mask_c;
			const Eigen::ArrayXd tan_norm = (tan_x.square() + tan_y.square()).sqrt();
			const Eigen::ArrayXd valid_tan = (tan_norm > 1e-6).cast<double>();
			Eigen::ArrayXd dir_cost = Eigen::ArrayXd::Ones(kBatch);
			{
				const Eigen::ArrayXd denom = tan_norm.max(1e-12);
				Eigen::ArrayXd tx = tan_x / denom;
				Eigen::ArrayXd ty = tan_y / denom;
				Eigen::ArrayXd dot = (tx * pref_dir.x() + ty * pref_dir.y());
				dot = dot.max(-1.0).min(1.0);
				dir_cost = (1.0 - dot) * valid_tan + dir_cost * (1.0 - valid_tan);
			}

			// endpoint costs
			const Eigen::ArrayXd ex = end_xy_mat.row(0).array() - p_virtual.x();
			const Eigen::ArrayXd ey = end_xy_mat.row(1).array() - p_virtual.y();
			const double pref_perp_x = -pref_dir.y();
			const double pref_perp_y = pref_dir.x();
			const Eigen::ArrayXd end_long = (ex * pref_dir.x() + ey * pref_dir.y()).abs();
			const Eigen::ArrayXd end_lat = (ex * pref_perp_x + ey * pref_perp_y).abs();
			const Eigen::ArrayXd end_cost_a = (w_dir * end_lat + w_end * end_long);
			const Eigen::ArrayXd end_dist_c = (ex.square() + ey.square()).sqrt();
			const Eigen::ArrayXd end_cost_c = (w_end * end_dist_c);
			Eigen::ArrayXd end_cost = end_cost_a * mask_a + end_cost_c * mask_c;

			Eigen::ArrayXd total_cost = w_collision * coll_norm_vec + w_kin * kin_cost_vec + w_dir * dir_cost + end_cost;
			total_cost = valid_plan.select(total_cost, Eigen::ArrayXd::Constant(kBatch, std::numeric_limits<double>::infinity()));

			// Debug: dump mixed batch samples and costs (avoidance + collision)
			{
				const double z_dbg = p0.z();
				const std::string fname = resolveDataCsvPathIntention(std::string("mixed_batch_samples_costs_.csv"));
				std::ofstream fout(fname, std::ios::out);
				if (fout.is_open()) {
					fout << std::setprecision(6) << std::fixed;
					fout << "plan_id,type,sample_idx,phase,local_idx,x,y,z,collision_cost,kin_cost,dir_cost,end_cost,total_cost\n";
					for (int bi = 0; bi < kBatch; ++bi) {
						const bool is_collision = (bi >= kAvoidN);
						const double c_cost = static_cast<double>(coll_norm_vec(bi));
						const double k_cost = static_cast<double>(kin_cost_vec(bi));
						const double d_cost = static_cast<double>(dir_cost(bi));
						const double e_cost = static_cast<double>(end_cost(bi));
						const double tot = static_cast<double>(total_cost(bi));

						if (!is_collision) {
							for (int m = 0; m < M; ++m) {
								fout << bi << ",avoid," << m << ",pre," << m << ","
									 << X_avoid(m, bi) << "," << Y_avoid(m, bi) << "," << z_dbg << ","
									 << c_cost << "," << k_cost << "," << d_cost << "," << e_cost << "," << tot << "\n";
							}
						} else {
							const int ci = bi - kAvoidN;
							int sample_idx = 0;
							for (int m = 0; m < Mpre_dbg; ++m) {
								fout << bi << ",collision," << sample_idx++ << ",pre," << m << ","
									 << Xpre_dbg(m, ci) << "," << Ypre_dbg(m, ci) << "," << z_dbg << ","
									 << c_cost << "," << k_cost << "," << d_cost << "," << e_cost << "," << tot << "\n";
							}
							for (int m = 0; m < Mpost_dbg; ++m) {
								fout << bi << ",collision," << sample_idx++ << ",post," << m << ","
									 << Xpost_dbg(m, ci) << "," << Ypost_dbg(m, ci) << "," << z_dbg << ","
									 << c_cost << "," << k_cost << "," << d_cost << "," << e_cost << "," << tot << "\n";
							}
						}
					}
					fout.close();
					ROS_INFO("Saved mixed batch samples+costs to %s", fname.c_str());
				} else {
					ROS_WARN("Failed to open %s for writing mixed batch samples+costs", fname.c_str());
				}
			}
			// 忽略所有避碰轨迹，进行碰撞debug
			const double INF_D = std::numeric_limits<double>::infinity();
			if (kAvoidN > 0) {
				total_cost.head(kAvoidN).setConstant(INF_D); // 忽略所有避碰候选
			}

			Eigen::Index idx = 0;
			total_cost.minCoeff(&idx);
			best_i = static_cast<int>(idx);
		}

		// If collision plan selected, optionally return CollisionTrajectory metadata.
		if (out_collision_traj && out_has_collision_traj && plans[best_i].trajectory_mode == quadrotor_msgs::TrajectoryPlan::MODE_COLLISION)
		{
			const int bi = best_i;
			const int ci = bi - kAvoidN;
			if (ci >= 0 && ci < kCollN && !samples.empty())
			{
				quadrotor_msgs::CollisionTrajectory ct;
				ct.header.stamp = ros::Time::now();
				ct.header.frame_id = "world";
				ct.trajectory_points.clear();
				ct.collision_events.clear();
				ct.segment_times.clear();

				// Provide post-collision trajectory points using two-point min-jerk with *optimized* duration.
				// NOTE: Keep the original endpoint position model unchanged; only the time is no longer fixed.
				const double z = preferred_target.z();
				const int pt_i = ci / 9; // [0..7]
				Eigen::Vector3d p_coll = samples[pt_i];
				p_coll.z() = z;
				Eigen::Vector3d v_post = v_post_world[ci];
				v_post.z() = 0.0;
				if (!v_post.allFinite()) v_post = Eigen::Vector3d::Zero();
				const double T_post = std::max(0.0, T_post_s[ci]);

				// Keep the original displacement model used by buildPostCollisionMinJerkDenseSamples:
				// average speed = v over T_post => dp = v_post * T_post.
				Eigen::Vector3d p_end = p_coll + v_post * T_post; // * 0.5 removed for consistency
				p_end.z() = z;
				Eigen::Vector3d dir_post = (p_end - p_coll);
				dir_post.z() = 0.0;
				if (dir_post.head<2>().norm() > 1e-6) dir_post.normalize();
				else dir_post = Eigen::Vector3d(1.0, 0.0, 0.0);

				auto post_plan = makeTwoPointMinJerkPlan(p_coll, p_end, dir_post, quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
				ct.trajectory_points = post_plan.waypoints;
				ct.segment_times = post_plan.segment_times;

				quadrotor_msgs::CollisionEvent ev;
				ev.collision_point.x = p_coll.x();
				ev.collision_point.y = p_coll.y();
				ev.collision_point.z = p_coll.z();
				ev.collision_time = pre_ratio[ci] * SAMPLE_FORWARD_FIXED_DURATION_S;
				// derive given_yaw from q_used = q_curr * yaw_body (reconstruct here)
				{
					Eigen::Quaterniond q_curr_local = Eigen::Quaterniond(pose.pose.orientation.w,
											 pose.pose.orientation.x,
											 pose.pose.orientation.y,
											 pose.pose.orientation.z);
					const int bi_input = kAvoidN + ci;
					size_t base_input = static_cast<size_t>(bi_input) * 14u;
					double cos_yaw = 1.0, sin_yaw = 0.0;
					if (base_input + 9u < input_raw.size()) {
						// yaw_vector stored as (cos, sin) in the new student input ordering
						cos_yaw = static_cast<double>(input_raw[base_input + 8]);
						sin_yaw = static_cast<double>(input_raw[base_input + 9]);
					}
					double yaw_body = std::atan2(sin_yaw, cos_yaw);
					Eigen::Quaterniond q_yaw = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()));
					Eigen::Quaterniond q_used_local = q_curr_local * q_yaw;
					// compute yaw from q_used_local
					double qw = q_used_local.w();
					double qx = q_used_local.x();
					double qy = q_used_local.y();
					double qz = q_used_local.z();
					double siny_cosp = 2.0 * (qw * qz + qx * qy);
					double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
					double yaw_used = std::atan2(siny_cosp, cosy_cosp);
					ev.given_yaw = yaw_used;
				}

				ev.pre_collision_velocity.x = v_pre_world_xy[ci].x();
				ev.pre_collision_velocity.y = v_pre_world_xy[ci].y();
				ev.pre_collision_velocity.z = 0.0;
				ev.post_collision_velocity.x = v_post_world[ci].x();
				ev.post_collision_velocity.y = v_post_world[ci].y();
				ev.post_collision_velocity.z = v_post_world[ci].z();

				ev.plane_normal.x = plane_n_unit_flipped.x();
				ev.plane_normal.y = plane_n_unit_flipped.y();
				ev.plane_normal.z = plane_n_unit_flipped.z();
				ev.plane_d = plane_d;

				ct.collision_events.push_back(ev);
				*out_collision_traj = ct;
				*out_has_collision_traj = true;
			}
		}

		// Convert best plan to discrete waypoints right before publishing:
		// - avoidance: full duration 4s
		// - collision: only pre-collision time window [0, pre_ratio*4s]
		const auto &best = plans[best_i];
		if (!best.waypoints.empty())
		{
			if (best.trajectory_mode == quadrotor_msgs::TrajectoryPlan::MODE_COLLISION)
			{
				const int ci = best_i - kAvoidN;
				const double r = (ci >= 0 && ci < kCollN) ? clamp(pre_ratio[ci], 0.0, 1.0) : 0.5;
				// Convert pre-collision B-spline control plan to discrete waypoints (pre segment)
				auto pre_plan = convertBsplineCtrlPlanToWaypointsMethod2Var(best, preferred_dir_unit, r * SAMPLE_FORWARD_FIXED_DURATION_S, r);

				// Ensure pre_plan contains at least one interior point (so it is not only start+end).
				// If only two waypoints exist, insert a midpoint and split the last segment time accordingly.
				if (pre_plan.waypoints.size() == 2) {
					geometry_msgs::Point p0 = pre_plan.waypoints.front();
					geometry_msgs::Point p1 = pre_plan.waypoints.back();
					geometry_msgs::Point mid;
					mid.x = 0.5 * (p0.x + p1.x);
					mid.y = 0.5 * (p0.y + p1.y);
					mid.z = 0.5 * (p0.z + p1.z);
					// insert midpoint
					pre_plan.waypoints.insert(pre_plan.waypoints.begin() + 1, mid);
					// duplicate heading from first waypoint (safe fallback)
					geometry_msgs::Vector3 mid_h = pre_plan.headings.empty() ? geometry_msgs::Vector3() : pre_plan.headings.front();
					pre_plan.headings.insert(pre_plan.headings.begin() + 1, mid_h);
					// insert position constraint false for the inserted middle point
					if (pre_plan.position_constraints.size() >= 1)
						pre_plan.position_constraints.insert(pre_plan.position_constraints.begin() + 1, false);
					else
						pre_plan.position_constraints.push_back(false);

					// Adjust segment_times: split the existing last dt into two halves (or create two small dts)
					if (!pre_plan.segment_times.empty()) {
						double dt = pre_plan.segment_times.back();
						pre_plan.segment_times.pop_back();
						double dt1 = std::max(1e-6, 0.5 * dt);
						double dt2 = dt - dt1;
						if (!std::isfinite(dt2) || dt2 <= 1e-6) dt2 = dt1;
						pre_plan.segment_times.push_back(dt1);
						pre_plan.segment_times.push_back(dt2);
					} else {
						pre_plan.segment_times.push_back(1e-3);
						pre_plan.segment_times.push_back(1e-3);
					}
				}

				return pre_plan;
			}
			return convertClampedUniformBsplineCtrlPlanToWaypointsMethod2(best, preferred_dir_unit, SAMPLE_FORWARD_FIXED_DURATION_S);
		}
		return best;
	}

	return makeEmptyPlan(quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
}

} // namespace sample_forward


#endif