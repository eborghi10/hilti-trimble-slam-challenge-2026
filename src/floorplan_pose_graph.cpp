/**
 * Phase 3B: Pose-graph optimization with EDT map factors (Ceres-based).
 *
 * Post-processes a Phase 3A TUM trajectory (already in floorplan frame) to correct
 * VIO drift using floorplan geometry constraints.
 *
 * Cost functions:
 *   - Anchor prior: strong prior at t=10005s (GT pose from init_gt_poses.csv)
 *   - Odometry factors: VIO relative pose between consecutive nodes
 *   - EDT map factors: exp(-d/sigma) penalty for poses near/inside walls
 *   - Smoothness: second-derivative penalty on correction spline
 *
 * Usage:
 *   floorplan_pose_graph <input_tum> <floorplan_png> <run_name> <output_tum> [options]
 *
 * Dependencies: Ceres 2.x, OpenCV (imread, distanceTransform), Eigen3
 */

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>

// ─── Configuration ───────────────────────────────────────────────────────────

struct Config {
  double node_spacing_sec = 0.5;   // One node every 0.5s
  double anchor_weight = 1000.0;
  double odom_weight_xy = 50.0;
  double odom_weight_yaw = 100.0;
  double map_weight = 5.0;
  double edt_margin = 0.05;       // meters — barrier active within this distance of wall
  double edt_threshold = 0.5;     // meters — only add factor if initial pose < threshold
  double smoothness_weight = 10.0;
  double resolution = 0.01;       // floorplan resolution m/pixel
};

// ─── Data structures ─────────────────────────────────────────────────────────

struct TumPose {
  double timestamp;
  double tx, ty, tz;
  double qx, qy, qz, qw;
};

struct Node2D {
  double timestamp;
  double x, y, yaw;    // Original (uncorrected) pose from Phase 3A
};

// ─── Utility functions ───────────────────────────────────────────────────────

static double quat_to_yaw(double qx, double qy, double qz, double qw) {
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

static double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static std::vector<TumPose> load_tum(const std::string& path) {
  std::vector<TumPose> poses;
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open trajectory: " + path);
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    TumPose p;
    if (ss >> p.timestamp >> p.tx >> p.ty >> p.tz >> p.qx >> p.qy >> p.qz >> p.qw) {
      poses.push_back(p);
    }
  }
  return poses;
}

struct GtAnchor {
  double timestamp;
  double x, y, yaw;
};

static GtAnchor load_anchor(const std::string& run_name, const std::string& csv_path) {
  std::ifstream f(csv_path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open: " + csv_path);
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    // Parse CSV: name, floorplan, timestamp, tx, ty, tz, qx, qy, qz, qw
    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
      // Trim whitespace
      size_t start = token.find_first_not_of(" \t");
      size_t end = token.find_last_not_of(" \t");
      if (start != std::string::npos)
        tokens.push_back(token.substr(start, end - start + 1));
      else
        tokens.push_back("");
    }
    if (tokens.size() < 10) continue;
    if (tokens[0] != run_name) continue;

    GtAnchor a;
    a.timestamp = std::stod(tokens[2]);
    a.x = std::stod(tokens[3]);
    a.y = std::stod(tokens[4]);
    double qx = std::stod(tokens[6]);
    double qy = std::stod(tokens[7]);
    double qz = std::stod(tokens[8]);
    double qw = std::stod(tokens[9]);
    a.yaw = quat_to_yaw(qx, qy, qz, qw);
    return a;
  }
  throw std::runtime_error("Run '" + run_name + "' not found in " + csv_path);
}

// ─── EDT (Euclidean Distance Transform) ─────────────────────────────────────

class EdtMap {
public:
  EdtMap() = default;

  void load(const std::string& png_path, double resolution) {
    resolution_ = resolution;
    cv::Mat img = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      throw std::runtime_error("Cannot read floorplan: " + png_path);
    }

    // Binary: walls (dark pixels) = 0, free space (light pixels) = 255
    cv::Mat binary;
    cv::threshold(img, binary, 128, 255, cv::THRESH_BINARY);

    // Distance transform: distance from each free pixel to nearest wall (in pixels)
    cv::Mat edt_pixels;
    cv::distanceTransform(binary, edt_pixels, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // Flip vertically (image row 0 = top, but floorplan y=0 = bottom)
    cv::flip(edt_pixels, edt_pixels, 0);

    // Convert to meters
    edt_pixels.convertTo(edt_, CV_64F, resolution);

    width_ = edt_.cols;
    height_ = edt_.rows;

    std::cout << "  EDT: " << width_ << "x" << height_ << " px, "
              << "extent=" << width_ * resolution_ << "x" << height_ * resolution_ << " m"
              << std::endl;
  }

  // Query with bilinear interpolation (for smooth Ceres gradients)
  double query(double x_m, double y_m) const {
    double col = x_m / resolution_;
    double row = y_m / resolution_;

    if (col < 0 || col >= width_ - 1 || row < 0 || row >= height_ - 1) {
      return 10.0;  // Out of bounds → large distance (no penalty)
    }

    // Bilinear interpolation
    int c0 = static_cast<int>(col);
    int r0 = static_cast<int>(row);
    double fc = col - c0;
    double fr = row - r0;

    double v00 = edt_.at<double>(r0, c0);
    double v01 = edt_.at<double>(r0, c0 + 1);
    double v10 = edt_.at<double>(r0 + 1, c0);
    double v11 = edt_.at<double>(r0 + 1, c0 + 1);

    return (1 - fr) * ((1 - fc) * v00 + fc * v01) +
           fr * ((1 - fc) * v10 + fc * v11);
  }

  double resolution() const { return resolution_; }
  int width() const { return width_; }
  int height() const { return height_; }

private:
  cv::Mat edt_;        // CV_64F, distance in meters
  double resolution_;
  int width_ = 0, height_ = 0;
};

// ─── Ceres Cost Functions (correction-based formulation) ─────────────────────
// Optimization variable: δ[i] = [δx, δy, δyaw] per node
// Corrected pose: x_corr = x_orig + δx, y_corr = y_orig + δy, yaw_corr = yaw_orig + δyaw

// Anchor prior: penalizes corrected pose deviation from GT
struct AnchorCost {
  AnchorCost(double orig_x, double orig_y, double orig_yaw,
             double gt_x, double gt_y, double gt_yaw, double weight)
    : orig_x_(orig_x), orig_y_(orig_y), orig_yaw_(orig_yaw),
      gt_x_(gt_x), gt_y_(gt_y), gt_yaw_(gt_yaw), weight_(std::sqrt(weight)) {}

  template <typename T>
  bool operator()(const T* const delta, T* residual) const {
    // corrected = original + delta
    T cx = T(orig_x_) + delta[0];
    T cy = T(orig_y_) + delta[1];
    T cyaw = T(orig_yaw_) + delta[2];

    residual[0] = weight_ * (cx - T(gt_x_));
    residual[1] = weight_ * (cy - T(gt_y_));
    T dyaw = cyaw - T(gt_yaw_);
    residual[2] = weight_ * ceres::atan2(ceres::sin(dyaw), ceres::cos(dyaw));
    return true;
  }

  static ceres::CostFunction* Create(double orig_x, double orig_y, double orig_yaw,
                                      double gt_x, double gt_y, double gt_yaw, double weight) {
    return new ceres::AutoDiffCostFunction<AnchorCost, 3, 3>(
      new AnchorCost(orig_x, orig_y, orig_yaw, gt_x, gt_y, gt_yaw, weight));
  }

  double orig_x_, orig_y_, orig_yaw_;
  double gt_x_, gt_y_, gt_yaw_, weight_;
};

// Odometry factor: relative pose consistency between corrected poses
struct OdometryCost {
  OdometryCost(double orig_xi, double orig_yi, double orig_yawi,
               double orig_xj, double orig_yj, double orig_yawj,
               double weight_xy, double weight_yaw)
    : orig_xi_(orig_xi), orig_yi_(orig_yi), orig_yawi_(orig_yawi),
      orig_xj_(orig_xj), orig_yj_(orig_yj), orig_yawj_(orig_yawj),
      weight_xy_(std::sqrt(weight_xy)), weight_yaw_(std::sqrt(weight_yaw)) {
    // Precompute expected relative pose from original trajectory
    double cos_yaw = std::cos(orig_yawi);
    double sin_yaw = std::sin(orig_yawi);
    double dx_world = orig_xj - orig_xi;
    double dy_world = orig_yj - orig_yi;
    exp_dx_local_ = cos_yaw * dx_world + sin_yaw * dy_world;
    exp_dy_local_ = -sin_yaw * dx_world + cos_yaw * dy_world;
    exp_dyaw_ = orig_yawj - orig_yawi;
  }

  template <typename T>
  bool operator()(const T* const delta_i, const T* const delta_j, T* residual) const {
    // Corrected poses
    T xi = T(orig_xi_) + delta_i[0];
    T yi = T(orig_yi_) + delta_i[1];
    T yawi = T(orig_yawi_) + delta_i[2];
    T xj = T(orig_xj_) + delta_j[0];
    T yj = T(orig_yj_) + delta_j[1];
    T yawj = T(orig_yawj_) + delta_j[2];

    // Relative pose in corrected frame i
    T cos_yaw = ceres::cos(yawi);
    T sin_yaw = ceres::sin(yawi);
    T dx_world = xj - xi;
    T dy_world = yj - yi;
    T rel_x = cos_yaw * dx_world + sin_yaw * dy_world;
    T rel_y = -sin_yaw * dx_world + cos_yaw * dy_world;
    T rel_yaw = yawj - yawi;

    // Error: corrected relative pose should match VIO relative pose
    residual[0] = weight_xy_ * (rel_x - T(exp_dx_local_));
    residual[1] = weight_xy_ * (rel_y - T(exp_dy_local_));
    T dyaw_err = rel_yaw - T(exp_dyaw_);
    residual[2] = weight_yaw_ * ceres::atan2(ceres::sin(dyaw_err), ceres::cos(dyaw_err));
    return true;
  }

  static ceres::CostFunction* Create(double orig_xi, double orig_yi, double orig_yawi,
                                      double orig_xj, double orig_yj, double orig_yawj,
                                      double weight_xy, double weight_yaw) {
    return new ceres::AutoDiffCostFunction<OdometryCost, 3, 3, 3>(
      new OdometryCost(orig_xi, orig_yi, orig_yawi, orig_xj, orig_yj, orig_yawj,
                       weight_xy, weight_yaw));
  }

  double orig_xi_, orig_yi_, orig_yawi_;
  double orig_xj_, orig_yj_, orig_yawj_;
  double exp_dx_local_, exp_dy_local_, exp_dyaw_;
  double weight_xy_, weight_yaw_;
};

// EDT map factor: penalizes proximity to walls on corrected pose.
// Two modes:
//   margin > 0: barrier mode — linear penalty when EDT < margin
//   margin <= 0: attraction mode — residual = weight * exp(-EDT/sigma)
//                (gentle pull toward corridor centers)
struct EdtCost {
  EdtCost(const EdtMap* edt, double orig_x, double orig_y, double margin, double weight)
    : edt_(edt), orig_x_(orig_x), orig_y_(orig_y), margin_(margin), weight_(std::sqrt(weight)) {}

  bool operator()(const double* const delta, double* residual) const {
    double x = orig_x_ + delta[0];
    double y = orig_y_ + delta[1];
    double d = edt_->query(x, y);

    if (margin_ > 0.0) {
      // Barrier mode
      if (d >= margin_) {
        residual[0] = 0.0;
      } else {
        residual[0] = weight_ * (margin_ - d) / margin_;
      }
    } else {
      // Attraction mode: gentle exponential penalty
      double sigma = -margin_;  // use absolute value of margin as sigma
      residual[0] = weight_ * std::exp(-d / sigma);
    }
    return true;
  }

  static ceres::CostFunction* Create(const EdtMap* edt, double orig_x, double orig_y,
                                      double margin, double weight) {
    return new ceres::NumericDiffCostFunction<EdtCost, ceres::CENTRAL, 1, 3>(
      new EdtCost(edt, orig_x, orig_y, margin, weight));
  }

  const EdtMap* edt_;
  double orig_x_, orig_y_, margin_, weight_;
};

// Smoothness factor: penalizes second derivative of CORRECTIONS (not trajectory)
// Since corrections start at zero, this prevents rapid oscillations in δ
struct SmoothnessCost {
  SmoothnessCost(double dt_avg, double weight)
    : inv_dt2_(1.0 / (dt_avg * dt_avg)), weight_(std::sqrt(weight)) {}

  template <typename T>
  bool operator()(const T* const prev, const T* const curr, const T* const next,
                  T* residual) const {
    // Second derivative of corrections: (δ[i+1] - 2*δ[i] + δ[i-1]) / dt^2
    residual[0] = weight_ * T(inv_dt2_) * (next[0] - T(2.0) * curr[0] + prev[0]);
    residual[1] = weight_ * T(inv_dt2_) * (next[1] - T(2.0) * curr[1] + prev[1]);
    residual[2] = weight_ * T(inv_dt2_) * (next[2] - T(2.0) * curr[2] + prev[2]);
    return true;
  }

  static ceres::CostFunction* Create(double dt_avg, double weight) {
    return new ceres::AutoDiffCostFunction<SmoothnessCost, 3, 3, 3, 3>(
      new SmoothnessCost(dt_avg, weight));
  }

  double inv_dt2_, weight_;
};

// ─── Pose Graph Builder & Optimizer ──────────────────────────────────────────

static void run_optimization(const std::vector<TumPose>& poses_full,
                             const EdtMap& edt_map,
                             const GtAnchor& anchor,
                             const Config& cfg,
                             const std::string& output_path) {
  // Build nodes: subsample trajectory at node_spacing_sec
  double t_start = poses_full.front().timestamp;
  double t_end = poses_full.back().timestamp;
  double duration = t_end - t_start;

  std::vector<Node2D> nodes;
  std::vector<double> full_yaws(poses_full.size());
  for (size_t i = 0; i < poses_full.size(); i++) {
    full_yaws[i] = quat_to_yaw(poses_full[i].qx, poses_full[i].qy,
                                poses_full[i].qz, poses_full[i].qw);
  }

  // Unwrap yaw for interpolation
  for (size_t i = 1; i < full_yaws.size(); i++) {
    double diff = full_yaws[i] - full_yaws[i-1];
    if (diff > M_PI) full_yaws[i] -= 2.0 * M_PI;
    else if (diff < -M_PI) full_yaws[i] += 2.0 * M_PI;
  }

  // Create nodes at regular intervals
  for (double t = t_start; t <= t_end; t += cfg.node_spacing_sec) {
    // Find interpolation indices
    size_t idx = 0;
    for (size_t i = 1; i < poses_full.size(); i++) {
      if (poses_full[i].timestamp >= t) {
        idx = i - 1;
        break;
      }
      idx = i;
    }
    if (idx >= poses_full.size() - 1) idx = poses_full.size() - 2;

    double t0 = poses_full[idx].timestamp;
    double t1 = poses_full[idx + 1].timestamp;
    double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
    alpha = std::clamp(alpha, 0.0, 1.0);

    Node2D node;
    node.timestamp = t;
    node.x = (1.0 - alpha) * poses_full[idx].tx + alpha * poses_full[idx + 1].tx;
    node.y = (1.0 - alpha) * poses_full[idx].ty + alpha * poses_full[idx + 1].ty;
    node.yaw = (1.0 - alpha) * full_yaws[idx] + alpha * full_yaws[idx + 1];
    nodes.push_back(node);
  }

  size_t N = nodes.size();

  // Find anchor node
  size_t anchor_idx = 0;
  double best_dt = 1e9;
  for (size_t i = 0; i < N; i++) {
    double dt = std::abs(nodes[i].timestamp - anchor.timestamp);
    if (dt < best_dt) {
      best_dt = dt;
      anchor_idx = i;
    }
  }

  std::cout << "  Nodes: " << N << " (spacing " << cfg.node_spacing_sec << "s, "
            << "duration " << duration << "s)" << std::endl;
  std::cout << "  Anchor at node " << anchor_idx
            << " (t=" << std::fixed << std::setprecision(3) << nodes[anchor_idx].timestamp << "s)"
            << std::endl;

  // Optimization variables: δ[i] = [δx, δy, δyaw] corrections per node
  // Initialized to zero (no correction)
  std::vector<std::array<double, 3>> deltas(N, {0.0, 0.0, 0.0});

  // Build Ceres problem
  ceres::Problem problem;

  // 1. Anchor prior (corrected pose should match GT)
  problem.AddResidualBlock(
    AnchorCost::Create(nodes[anchor_idx].x, nodes[anchor_idx].y, nodes[anchor_idx].yaw,
                       anchor.x, anchor.y, anchor.yaw, cfg.anchor_weight),
    nullptr, deltas[anchor_idx].data());

  // 2. Odometry factors (corrected relative pose should match VIO relative pose)
  for (size_t i = 0; i < N - 1; i++) {
    problem.AddResidualBlock(
      OdometryCost::Create(nodes[i].x, nodes[i].y, nodes[i].yaw,
                           nodes[i+1].x, nodes[i+1].y, nodes[i+1].yaw,
                           cfg.odom_weight_xy, cfg.odom_weight_yaw),
      nullptr, deltas[i].data(), deltas[i + 1].data());
  }

  // 3. EDT map factors — barrier penalty for wall penetration
  size_t map_factors = 0;
  for (size_t i = 0; i < N; i++) {
    problem.AddResidualBlock(
      EdtCost::Create(&edt_map, nodes[i].x, nodes[i].y, cfg.edt_margin, cfg.map_weight),
      nullptr, deltas[i].data());
    double d = edt_map.query(nodes[i].x, nodes[i].y);
    if (d < cfg.edt_margin) map_factors++;
  }
  std::cout << "  Map factors: " << N << " total, " << map_factors
            << " initially active (within " << cfg.edt_margin << "m of wall)" << std::endl;

  // 4. Smoothness factors (penalizes second derivative of corrections)
  for (size_t i = 1; i < N - 1; i++) {
    double dt_prev = nodes[i].timestamp - nodes[i - 1].timestamp;
    double dt_next = nodes[i + 1].timestamp - nodes[i].timestamp;
    double dt_avg = 0.5 * (dt_prev + dt_next);

    problem.AddResidualBlock(
      SmoothnessCost::Create(dt_avg, cfg.smoothness_weight),
      nullptr, deltas[i - 1].data(), deltas[i].data(), deltas[i + 1].data());
  }

  // Solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 200;
  options.function_tolerance = 1e-10;
  options.gradient_tolerance = 1e-8;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 4;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "\n  " << summary.BriefReport() << std::endl;

  // Compute correction statistics
  double max_dx = 0, max_dy = 0, max_dyaw = 0;
  double sum_disp = 0;
  for (size_t i = 0; i < N; i++) {
    max_dx = std::max(max_dx, std::abs(deltas[i][0]));
    max_dy = std::max(max_dy, std::abs(deltas[i][1]));
    max_dyaw = std::max(max_dyaw, std::abs(deltas[i][2]));
    sum_disp += std::sqrt(deltas[i][0] * deltas[i][0] + deltas[i][1] * deltas[i][1]);
  }
  std::cout << "  Max correction: dx=" << max_dx << "m, dy=" << max_dy
            << "m, dyaw=" << max_dyaw * 180.0 / M_PI << "°" << std::endl;
  std::cout << "  Mean displacement: " << sum_disp / N << "m" << std::endl;

  // Interpolate corrections back to full-rate trajectory
  std::vector<double> node_times(N), corr_x(N), corr_y(N), corr_yaw(N);
  for (size_t i = 0; i < N; i++) {
    node_times[i] = nodes[i].timestamp;
    corr_x[i] = deltas[i][0];
    corr_y[i] = deltas[i][1];
    corr_yaw[i] = deltas[i][2];
  }

  // Linear interpolation for corrections (simple, fast, sufficient for 0.5s spacing)
  std::ofstream out(output_path);
  if (!out.is_open()) {
    throw std::runtime_error("Cannot open output file: " + output_path);
  }
  out << "# timestamp tx ty tz qx qy qz qw\n";
  out << std::fixed;

  size_t node_lo = 0;
  for (size_t i = 0; i < poses_full.size(); i++) {
    double t = poses_full[i].timestamp;

    // Find bracketing nodes
    while (node_lo < N - 2 && node_times[node_lo + 1] < t) {
      node_lo++;
    }
    size_t node_hi = std::min(node_lo + 1, N - 1);

    double alpha = 0.0;
    if (node_hi > node_lo) {
      alpha = (t - node_times[node_lo]) / (node_times[node_hi] - node_times[node_lo]);
      alpha = std::clamp(alpha, 0.0, 1.0);
    }

    double dx = (1.0 - alpha) * corr_x[node_lo] + alpha * corr_x[node_hi];
    double dy = (1.0 - alpha) * corr_y[node_lo] + alpha * corr_y[node_hi];
    double dyaw = (1.0 - alpha) * corr_yaw[node_lo] + alpha * corr_yaw[node_hi];

    // Apply correction
    double new_x = poses_full[i].tx + dx;
    double new_y = poses_full[i].ty + dy;
    double new_z = poses_full[i].tz;  // z pass-through

    // Apply yaw correction to quaternion (rotation around z in world frame)
    double half_dyaw = dyaw * 0.5;
    double dq_w = std::cos(half_dyaw);
    double dq_z = std::sin(half_dyaw);
    // q_corrected = dq * q_original
    double oqx = poses_full[i].qx, oqy = poses_full[i].qy;
    double oqz = poses_full[i].qz, oqw = poses_full[i].qw;
    double new_qw = dq_w * oqw - dq_z * oqz;
    double new_qx = dq_w * oqx - dq_z * oqy;
    double new_qy = dq_w * oqy + dq_z * oqx;
    double new_qz = dq_w * oqz + dq_z * oqw;
    double norm = std::sqrt(new_qw * new_qw + new_qx * new_qx +
                            new_qy * new_qy + new_qz * new_qz);
    new_qx /= norm; new_qy /= norm; new_qz /= norm; new_qw /= norm;

    out << std::setprecision(9) << t << " "
        << std::setprecision(12)
        << new_x << " " << new_y << " " << new_z << " "
        << new_qx << " " << new_qy << " " << new_qz << " " << new_qw << "\n";
  }

  out.close();
  std::cout << "  Output: " << poses_full.size() << " corrected poses → " << output_path << std::endl;
}

// ─── Main ────────────────────────────────────────────────────────────────────

static void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " <input_tum> <floorplan_png> <run_name> <output_tum> [options]\n"
            << "\nOptions:\n"
            << "  --gt-csv <path>         Path to init_gt_poses.csv\n"
            << "  --resolution <m/px>     Floorplan resolution (default: 0.01)\n"
            << "  --node-spacing <sec>    Pose graph node spacing (default: 0.5)\n"
            << "  --map-weight <w>        EDT map factor weight (default: 5.0)\n"
            << "  --edt-margin <m>        EDT barrier margin (default: 0.15)\n"
            << "  --anchor-weight <w>     Anchor prior weight (default: 1000.0)\n"
            << "  --odom-weight-xy <w>    Odometry translation weight (default: 50.0)\n"
            << "  --odom-weight-yaw <w>   Odometry yaw weight (default: 100.0)\n"
            << "  --smoothness-weight <w> Smoothness penalty weight (default: 10.0)\n";
}

int main(int argc, char** argv) {
  if (argc < 5) {
    print_usage(argv[0]);
    return 1;
  }

  std::string input_tum = argv[1];
  std::string floorplan_png = argv[2];
  std::string run_name = argv[3];
  std::string output_tum = argv[4];
  std::string gt_csv;

  Config cfg;

  // Parse optional arguments
  for (int i = 5; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--gt-csv" && i + 1 < argc) gt_csv = argv[++i];
    else if (arg == "--resolution" && i + 1 < argc) cfg.resolution = std::stod(argv[++i]);
    else if (arg == "--node-spacing" && i + 1 < argc) cfg.node_spacing_sec = std::stod(argv[++i]);
    else if (arg == "--map-weight" && i + 1 < argc) cfg.map_weight = std::stod(argv[++i]);
    else if (arg == "--edt-margin" && i + 1 < argc) cfg.edt_margin = std::stod(argv[++i]);
    else if (arg == "--anchor-weight" && i + 1 < argc) cfg.anchor_weight = std::stod(argv[++i]);
    else if (arg == "--odom-weight-xy" && i + 1 < argc) cfg.odom_weight_xy = std::stod(argv[++i]);
    else if (arg == "--odom-weight-yaw" && i + 1 < argc) cfg.odom_weight_yaw = std::stod(argv[++i]);
    else if (arg == "--smoothness-weight" && i + 1 < argc) cfg.smoothness_weight = std::stod(argv[++i]);
    else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }

  // Find GT CSV
  if (gt_csv.empty()) {
    // Try standard locations
    std::vector<std::string> candidates = {
      "groundtruth/init_gt_poses.csv",
      "/ros2_ws/install/challenge_tools_ros/share/challenge_tools_ros/groundtruth/init_gt_poses.csv",
      "/ros2_ws/src/hilti-trimble-slam-challenge-2026/groundtruth/init_gt_poses.csv",
    };
    for (const auto& c : candidates) {
      std::ifstream test(c);
      if (test.good()) {
        gt_csv = c;
        break;
      }
    }
    if (gt_csv.empty()) {
      std::cerr << "ERROR: Cannot find init_gt_poses.csv. Specify with --gt-csv.\n";
      return 1;
    }
  }

  std::cout << "═══ Phase 3B: Ceres Pose Graph + EDT Map Factors ═══\n"
            << "  Input:      " << input_tum << "\n"
            << "  Floorplan:  " << floorplan_png << "\n"
            << "  Run:        " << run_name << "\n"
            << "  Output:     " << output_tum << "\n"
            << "  GT CSV:     " << gt_csv << "\n\n";

  // 1. Load trajectory
  std::cout << "[1/4] Loading trajectory..." << std::endl;
  auto poses = load_tum(input_tum);
  std::cout << "  Loaded " << poses.size() << " poses, t=["
            << std::fixed << std::setprecision(3)
            << poses.front().timestamp << ", " << poses.back().timestamp << "]s" << std::endl;

  // 2. Load floorplan EDT
  std::cout << "[2/4] Computing EDT from floorplan..." << std::endl;
  EdtMap edt_map;
  edt_map.load(floorplan_png, cfg.resolution);

  // 3. Load anchor
  std::cout << "[3/4] Loading GT anchor..." << std::endl;
  auto anchor = load_anchor(run_name, gt_csv);
  std::cout << "  Anchor: t=" << std::setprecision(6) << anchor.timestamp
            << "s, x=" << std::setprecision(3) << anchor.x
            << "m, y=" << anchor.y << "m, yaw=" << anchor.yaw * 180.0 / M_PI << "°"
            << std::endl;

  // 4. Optimize
  std::cout << "[4/4] Optimizing pose graph..." << std::endl;
  run_optimization(poses, edt_map, anchor, cfg, output_tum);

  std::cout << "\n═══ Done ═══\n";
  return 0;
}
