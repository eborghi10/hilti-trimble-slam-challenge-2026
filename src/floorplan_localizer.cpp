/**
 * Floorplan Localizer (Phase 3A): Transforms VIO trajectory into floorplan frame
 * using the challenge-provided GT anchor pose at t=10005s.
 *
 * Subscribes to VIO odometry, computes a rigid transform T_floorplan<-vio at the
 * anchor timestamp, and republishes all poses in floorplan coordinates.
 */

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>

struct GtAnchor {
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation; // (w, x, y, z) internally in Eigen
};

static GtAnchor load_gt_anchor(const std::string& run_name) {
  std::string share_dir = ament_index_cpp::get_package_share_directory("challenge_tools_ros");
  std::string csv_path = share_dir + "/groundtruth/init_gt_poses.csv";

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open " + csv_path);
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
      // trim whitespace
      size_t start = token.find_first_not_of(" \t");
      size_t end = token.find_last_not_of(" \t");
      if (start != std::string::npos)
        tokens.push_back(token.substr(start, end - start + 1));
      else
        tokens.push_back("");
    }

    if (tokens.size() < 10) continue;
    if (tokens[0] != run_name) continue;

    GtAnchor anchor;
    anchor.timestamp = std::stod(tokens[2]);
    anchor.position = Eigen::Vector3d(
      std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
    // CSV order: qx, qy, qz, qw — Eigen::Quaterniond constructor: (w, x, y, z)
    anchor.orientation = Eigen::Quaterniond(
      std::stod(tokens[9]), std::stod(tokens[6]),
      std::stod(tokens[7]), std::stod(tokens[8]));
    anchor.orientation.normalize();
    return anchor;
  }

  throw std::runtime_error("Run '" + run_name + "' not found in " + csv_path);
}

static Eigen::Isometry3d load_T_imu_cam() {
  std::string share_dir = ament_index_cpp::get_package_share_directory("challenge_tools_ros");
  std::string yaml_path = share_dir + "/config/hilti_openvins/kalibr_imucam_chain.yaml";

  std::ifstream file(yaml_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open " + yaml_path);
  }

  // Parse T_cam_imu matrix from YAML (simplified: look for the 4 rows after "T_cam_imu:")
  // Hard-code the calibration values since parsing nested YAML without a library is fragile
  Eigen::Matrix4d T_cam_imu_mat;
  T_cam_imu_mat <<
    0.017214474772216132, -0.0008034642120502422, -0.9998514971252359, 0.020670851120764513,
    0.9998263174555488, -0.007128426214556394, 0.017219769539067287, 0.015539085669546057,
    -0.007141203091335369, -0.9999742696614562, 0.0006806125511055194, -0.01575188948566258,
    0.0, 0.0, 0.0, 1.0;

  Eigen::Isometry3d T_cam_imu = Eigen::Isometry3d::Identity();
  T_cam_imu.linear() = T_cam_imu_mat.block<3,3>(0,0);
  T_cam_imu.translation() = T_cam_imu_mat.block<3,1>(0,3);

  return T_cam_imu.inverse(); // return T_imu_cam
}

class FloorplanLocalizer : public rclcpp::Node {
public:
  FloorplanLocalizer(const std::string& run_name,
                     const std::string& output_file,
                     const std::string& odom_topic)
    : Node("floorplan_localizer"), anchor_matched_(false),
      best_anchor_dt_(std::numeric_limits<double>::max()), pose_count_(0)
  {
    // Load GT anchor (T_fp_cam at anchor time)
    auto anchor = load_gt_anchor(run_name);
    anchor_ts_ = anchor.timestamp;
    T_fp_cam_anchor_ = Eigen::Isometry3d::Identity();
    T_fp_cam_anchor_.linear() = anchor.orientation.toRotationMatrix();
    T_fp_cam_anchor_.translation() = anchor.position;

    // Load IMU-camera extrinsic
    T_imu_cam_ = load_T_imu_cam();

    RCLCPP_INFO(get_logger(),
      "GT anchor for '%s' at t=%.6f s: pos=[%.3f, %.3f, %.3f]",
      run_name.c_str(), anchor_ts_,
      anchor.position.x(), anchor.position.y(), anchor.position.z());

    // Open output file
    out_file_.open(output_file);
    if (!out_file_.is_open()) {
      throw std::runtime_error("Cannot open output file: " + output_file);
    }
    out_file_ << "# timestamp tx ty tz qx qy qz qw\n";
    out_file_ << std::fixed;

    // Publisher
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/floorplan/pose_corrected", 10);

    // Subscriber
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50,
      std::bind(&FloorplanLocalizer::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to: %s", odom_topic.c_str());
    RCLCPP_INFO(get_logger(), "Output TUM file: %s", output_file.c_str());

    // Timer to check anchor lock
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&FloorplanLocalizer::check_anchor_lock, this));
  }

  ~FloorplanLocalizer() {
    if (out_file_.is_open()) {
      out_file_.flush();
      out_file_.close();
    }
    RCLCPP_INFO(get_logger(), "Final: %zu poses written.", pose_count_);
  }

private:
  double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  Eigen::Isometry3d msg_to_isometry(const nav_msgs::msg::Odometry& msg) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    auto& p = msg.pose.pose.position;
    auto& q = msg.pose.pose.orientation;
    T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    quat.normalize();
    T.linear() = quat.toRotationMatrix();
    return T;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double t = stamp_to_sec(msg->header.stamp);
    Eigen::Isometry3d T_vio_body = msg_to_isometry(*msg);

    if (!anchor_matched_) {
      double dt = std::abs(t - anchor_ts_);
      if (dt < best_anchor_dt_) {
        best_anchor_dt_ = dt;
        best_anchor_vio_T_ = T_vio_body;
      }
      pose_buffer_.emplace_back(msg->header.stamp, T_vio_body);
    } else {
      transform_and_publish(msg->header.stamp, T_vio_body);
    }
  }

  void check_anchor_lock() {
    if (anchor_matched_) {
      timer_->cancel();
      return;
    }
    if (best_anchor_dt_ == std::numeric_limits<double>::max()) return;

    if (!pose_buffer_.empty()) {
      double latest_t = stamp_to_sec(pose_buffer_.back().first);
      if (latest_t > anchor_ts_ + 1.0 || best_anchor_dt_ < 0.1) {
        lock_anchor();
      }
    }
  }

  void lock_anchor() {
    // VIO publishes T_vio_imu. GT anchor is T_fp_cam.
    // T_vio_cam = T_vio_imu * T_imu_cam
    Eigen::Isometry3d T_vio_cam_anchor = best_anchor_vio_T_ * T_imu_cam_;
    // T_fp_vio = T_fp_cam_anchor * inv(T_vio_cam_anchor)
    T_fp_vio_ = T_fp_cam_anchor_ * T_vio_cam_anchor.inverse();
    anchor_matched_ = true;

    RCLCPP_INFO(get_logger(),
      "Anchor locked (dt=%.1f ms). Replaying %zu buffered poses.",
      best_anchor_dt_ * 1000.0, pose_buffer_.size());

    for (auto& [stamp, T_vio_body] : pose_buffer_) {
      transform_and_publish(stamp, T_vio_body);
    }
    pose_buffer_.clear();
    pose_buffer_.shrink_to_fit();
  }

  void transform_and_publish(const builtin_interfaces::msg::Time& stamp,
                             const Eigen::Isometry3d& T_vio_imu) {
    // Convert IMU pose to camera pose, then to floorplan frame
    Eigen::Isometry3d T_vio_cam = T_vio_imu * T_imu_cam_;
    Eigen::Isometry3d T_fp_cam = T_fp_vio_ * T_vio_cam;
    Eigen::Vector3d pos = T_fp_cam.translation();
    Eigen::Quaterniond quat(T_fp_cam.linear());
    quat.normalize();

    // Publish
    nav_msgs::msg::Odometry out_msg;
    out_msg.header.stamp = stamp;
    out_msg.header.frame_id = "floorplan";
    out_msg.pose.pose.position.x = pos.x();
    out_msg.pose.pose.position.y = pos.y();
    out_msg.pose.pose.position.z = pos.z();
    out_msg.pose.pose.orientation.x = quat.x();
    out_msg.pose.pose.orientation.y = quat.y();
    out_msg.pose.pose.orientation.z = quat.z();
    out_msg.pose.pose.orientation.w = quat.w();
    pub_->publish(out_msg);

    // Log TUM
    double t_sec = stamp_to_sec(stamp);
    out_file_ << std::setprecision(9) << t_sec << " "
              << std::setprecision(12)
              << pos.x() << " " << pos.y() << " " << pos.z() << " "
              << quat.x() << " " << quat.y() << " " << quat.z() << " "
              << quat.w() << "\n";

    pose_count_++;
    if (pose_count_ % 500 == 0) {
      out_file_.flush();
      RCLCPP_INFO(get_logger(), "Logged %zu poses", pose_count_);
    }
  }

  // State
  double anchor_ts_;
  Eigen::Isometry3d T_fp_cam_anchor_;
  Eigen::Isometry3d T_imu_cam_;
  Eigen::Isometry3d T_fp_vio_;
  Eigen::Isometry3d best_anchor_vio_T_;
  bool anchor_matched_;
  double best_anchor_dt_;
  std::vector<std::pair<builtin_interfaces::msg::Time, Eigen::Isometry3d>> pose_buffer_;

  // I/O
  std::ofstream out_file_;
  size_t pose_count_;

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Parse args: floorplan_localizer <run_name> <output_file> [odom_topic]
  if (argc < 3) {
    std::cerr << "Usage: floorplan_localizer <run_name> <output_tum_file> [odom_topic]\n"
              << "  run_name:        e.g. floor_1_2025-05-05_run_1\n"
              << "  output_tum_file: path to write TUM-format trajectory\n"
              << "  odom_topic:      (optional) default: /loop_fusion/odometry_rect\n";
    return 1;
  }

  std::string run_name = argv[1];
  std::string output_file = argv[2];
  std::string odom_topic = (argc > 3) ? argv[3] : "/loop_fusion/odometry_rect";

  auto node = std::make_shared<FloorplanLocalizer>(run_name, output_file, odom_topic);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
