#include <Eigen/Geometry>

#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

std::string vector_to_string(const std::vector<double>& values, int precision = 6)
{
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i > 0)
      oss << ", ";
    oss << std::fixed << std::setprecision(precision) << values[i];
  }
  oss << "]";
  return oss.str();
}

std::vector<double> radians_to_degrees(const std::vector<double>& values)
{
  std::vector<double> out(values.size(), 0.0);
  for (std::size_t i = 0; i < values.size(); ++i)
    out[i] = values[i] * kRadToDeg;
  return out;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.linear() = q.toRotationMatrix();
  return transform;
}

std::string pose_to_string(const geometry_msgs::msg::Pose& pose)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "position(x,y,z)= [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]"
      << ", quaternion(x,y,z,w)= [" << pose.orientation.x << ", " << pose.orientation.y << ", "
      << pose.orientation.z << ", " << pose.orientation.w << "]";
  return oss.str();
}

double position_error(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientation_error_rad(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
{
  Eigen::Quaterniond qa(a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z);
  Eigen::Quaterniond qb(b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z);
  qa.normalize();
  qb.normalize();
  const double dot = std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

std::vector<double> absolute_joint_error(const std::vector<double>& a, const std::vector<double>& b)
{
  std::vector<double> out;
  const std::size_t n = std::min(a.size(), b.size());
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    out.push_back(std::abs(a[i] - b[i]));
  return out;
}

bool has_pose_inputs(const std::vector<double>& xyz, const std::vector<double>& quat)
{
  return xyz.size() == 3 && quat.size() == 4;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ik_fk_validator");
  auto logger = node->get_logger();

  node->declare_parameter<std::string>("group_name", "dummy_arm");
  node->declare_parameter<std::string>("tip_link", "");
  node->declare_parameter<std::string>("mode", "roundtrip");
  node->declare_parameter<std::string>("named_state", "reset");
  node->declare_parameter<std::vector<double>>("joint_values", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("target_pose_xyz", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("target_pose_quat_xyzw", std::vector<double>{});
  node->declare_parameter<double>("ik_timeout_s", 0.5);
  node->declare_parameter<int>("ik_attempts", 5);
  node->declare_parameter<bool>("enforce_bounds", true);

  const std::string group_name = node->get_parameter("group_name").as_string();
  std::string tip_link = node->get_parameter("tip_link").as_string();
  const std::string mode = node->get_parameter("mode").as_string();
  const std::string named_state = node->get_parameter("named_state").as_string();
  const auto joint_values_param = node->get_parameter("joint_values").as_double_array();
  const auto target_pose_xyz_param = node->get_parameter("target_pose_xyz").as_double_array();
  const auto target_pose_quat_param = node->get_parameter("target_pose_quat_xyzw").as_double_array();
  const double ik_timeout_s = node->get_parameter("ik_timeout_s").as_double();
  const int ik_attempts = node->get_parameter("ik_attempts").as_int();
  const bool enforce_bounds = node->get_parameter("enforce_bounds").as_bool();

  robot_model_loader::RobotModelLoader model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr model = model_loader.getModel();
  if (!model)
  {
    RCLCPP_ERROR(logger, "Failed to load robot model from parameter 'robot_description'.");
    rclcpp::shutdown();
    return 1;
  }

  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group_name);
  if (!jmg)
  {
    RCLCPP_ERROR(logger, "Joint model group '%s' was not found.", group_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (tip_link.empty())
  {
    const auto& links = jmg->getLinkModelNames();
    if (links.empty())
    {
      RCLCPP_ERROR(logger, "Joint model group '%s' has no links.", group_name.c_str());
      rclcpp::shutdown();
      return 1;
    }
    tip_link = links.back();
  }

  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  std::vector<double> input_joint_values;
  if (!joint_values_param.empty())
  {
    input_joint_values.assign(joint_values_param.begin(), joint_values_param.end());
    if (input_joint_values.size() != jmg->getVariableCount())
    {
      RCLCPP_ERROR(logger, "joint_values expects %zu values but got %zu.", jmg->getVariableCount(),
                   input_joint_values.size());
      rclcpp::shutdown();
      return 1;
    }
    state.setJointGroupPositions(jmg, input_joint_values);
  }
  else if (!named_state.empty())
  {
    state.setToDefaultValues(jmg, named_state);
    state.copyJointGroupPositions(jmg, input_joint_values);
  }
  else
  {
    state.copyJointGroupPositions(jmg, input_joint_values);
  }

  if (enforce_bounds)
    state.enforceBounds(jmg);
  state.update();

  const auto& joint_names = jmg->getVariableNames();
  RCLCPP_INFO(logger, "Model frame: %s", model->getModelFrame().c_str());
  RCLCPP_INFO(logger, "Group: %s | Tip link: %s | Mode: %s", group_name.c_str(), tip_link.c_str(), mode.c_str());
  RCLCPP_INFO(logger, "Seed / input joints (rad): %s", vector_to_string(input_joint_values).c_str());
  RCLCPP_INFO(logger, "Seed / input joints (deg): %s", vector_to_string(radians_to_degrees(input_joint_values), 3).c_str());
  for (std::size_t i = 0; i < joint_names.size() && i < input_joint_values.size(); ++i)
  {
    RCLCPP_INFO(logger, "  %s = %.6f rad (%.3f deg)", joint_names[i].c_str(), input_joint_values[i],
                input_joint_values[i] * kRadToDeg);
  }

  if (mode == "fk")
  {
    const auto pose = eigen_to_pose(state.getGlobalLinkTransform(tip_link));
    RCLCPP_INFO(logger, "FK result: %s", pose_to_string(pose).c_str());
    rclcpp::shutdown();
    return 0;
  }

  geometry_msgs::msg::Pose target_pose;
  if (mode == "roundtrip")
  {
    target_pose = eigen_to_pose(state.getGlobalLinkTransform(tip_link));
    RCLCPP_INFO(logger, "Roundtrip FK target pose: %s", pose_to_string(target_pose).c_str());
  }
  else if (mode == "ik")
  {
    if (!has_pose_inputs(target_pose_xyz_param, target_pose_quat_param))
    {
      RCLCPP_ERROR(logger,
                   "IK mode needs target_pose_xyz (3 values) and target_pose_quat_xyzw (4 values). Example: "
                   "target_pose_xyz:=[0.2,0.0,0.3] target_pose_quat_xyzw:=[0,0,0,1]");
      rclcpp::shutdown();
      return 1;
    }
    target_pose.position.x = target_pose_xyz_param[0];
    target_pose.position.y = target_pose_xyz_param[1];
    target_pose.position.z = target_pose_xyz_param[2];
    target_pose.orientation.x = target_pose_quat_param[0];
    target_pose.orientation.y = target_pose_quat_param[1];
    target_pose.orientation.z = target_pose_quat_param[2];
    target_pose.orientation.w = target_pose_quat_param[3];
    RCLCPP_INFO(logger, "IK target pose: %s", pose_to_string(target_pose).c_str());
  }
  else
  {
    RCLCPP_ERROR(logger, "Unsupported mode '%s'. Use fk, ik, or roundtrip.", mode.c_str());
    rclcpp::shutdown();
    return 1;
  }

  bool found_ik = false;
  for (int attempt = 0; attempt < std::max(1, ik_attempts); ++attempt)
  {
    moveit::core::RobotState ik_state(state);
    found_ik = ik_state.setFromIK(jmg, target_pose, tip_link, ik_timeout_s);
    if (found_ik)
    {
      state = ik_state;
      state.update();
      break;
    }
  }

  if (!found_ik)
  {
    RCLCPP_ERROR(logger, "IK failed after %d attempt(s), timeout %.3f s each.", std::max(1, ik_attempts), ik_timeout_s);
    rclcpp::shutdown();
    return 2;
  }

  std::vector<double> ik_joint_values;
  state.copyJointGroupPositions(jmg, ik_joint_values);
  const auto achieved_pose = eigen_to_pose(state.getGlobalLinkTransform(tip_link));

  RCLCPP_INFO(logger, "IK solution (rad): %s", vector_to_string(ik_joint_values).c_str());
  RCLCPP_INFO(logger, "IK solution (deg): %s", vector_to_string(radians_to_degrees(ik_joint_values), 3).c_str());
  RCLCPP_INFO(logger, "Achieved FK pose from IK solution: %s", pose_to_string(achieved_pose).c_str());
  RCLCPP_INFO(logger, "Pose position error: %.9f m", position_error(target_pose, achieved_pose));
  RCLCPP_INFO(logger, "Pose orientation error: %.9f rad (%.6f deg)", orientation_error_rad(target_pose, achieved_pose),
              orientation_error_rad(target_pose, achieved_pose) * kRadToDeg);

  if (mode == "roundtrip")
  {
    const auto joint_error = absolute_joint_error(input_joint_values, ik_joint_values);
    RCLCPP_INFO(logger, "Roundtrip absolute joint error (rad): %s", vector_to_string(joint_error).c_str());
    RCLCPP_INFO(logger, "Roundtrip absolute joint error (deg): %s",
                vector_to_string(radians_to_degrees(joint_error), 3).c_str());
  }

  rclcpp::shutdown();
  return 0;
}
