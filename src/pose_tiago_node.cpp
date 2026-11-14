#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

class PoseManager : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  PoseManager()
  : Node("pose_manager")
  {
    // Cargar el archivo de poses YAML
    std::string pkg_share = ament_index_cpp::get_package_share_directory("hri_perception_bridge");
    std::string yaml_path = pkg_share + "/config/robot_poses.yaml";
    
    load_poses_yaml(yaml_path);

    // Clientes de acción para los controladores del TIAGo
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/arm_controller/follow_joint_trajectory");
    torso_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/torso_controller/follow_joint_trajectory");
    head_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/head_controller/follow_joint_trajectory");
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/gripper_controller/follow_joint_trajectory");

    // Suscriptor para recibir el nombre de la pose
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/command_pose", 10,
      std::bind(&PoseManager::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Pose Manager listo (C++). Escuchando en /robot/command_pose");
  }

private:
  struct RobotPose {
    std::vector<double> arm_joints;
    std::vector<double> torso_joints;
    std::vector<double> head_joints;
    std::vector<double> gripper_joints;
  };

  std::unordered_map<std::string, RobotPose> poses_data_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr torso_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr head_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void load_poses_yaml(const std::string & path)
  {
    try {
      YAML::Node config = YAML::LoadFile(path);
      YAML::Node poses = config["poses"];

      for (YAML::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        std::string name = it->first.as<std::string>();
        YAML::Node data = it->second;
        RobotPose pose;

        if (data["arm_joints"]) {
          pose.arm_joints = data["arm_joints"].as<std::vector<double>>();
        }
        if (data["torso_lift_joint"]) {
          pose.torso_joints = {data["torso_lift_joint"].as<double>()};
        }
        if (data["head_1_joint"] && data["head_2_joint"]) {
          pose.head_joints = {data["head_1_joint"].as<double>(), data["head_2_joint"].as<double>()};
        } else if (data["head_joints"]) {
          pose.head_joints = data["head_joints"].as<std::vector<double>>();
        }
        if (data["gripper_joints"]) {
          pose.gripper_joints = data["gripper_joints"].as<std::vector<double>>();
        }

        poses_data_[name] = pose;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error leyendo %s: %s", path.c_str(), e.what());
    }
  }

  void pose_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string pose_name = msg->data;
    for (auto & c : pose_name) { c = std::tolower(c); }

    if (poses_data_.find(pose_name) == poses_data_.end()) {
      RCLCPP_WARN(this->get_logger(), "Pose '%s' no encontrada en YAML", pose_name.c_str());
      return;
    }

    const auto & pose = poses_data_[pose_name];
    RCLCPP_INFO(this->get_logger(), "Ejecutando pose: %s", pose_name.c_str());

    // 1. Enviar brazo
    if (!pose.arm_joints.empty()) {
      send_trajectory(
        arm_client_,
        {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"},
        pose.arm_joints, 3);
    }

    // 2. Enviar torso
    if (!pose.torso_joints.empty()) {
      send_trajectory(torso_client_, {"torso_lift_joint"}, pose.torso_joints, 2);
    }

    // 3. Enviar cabeza
    if (!pose.head_joints.empty()) {
      send_trajectory(head_client_, {"head_1_joint", "head_2_joint"}, pose.head_joints, 1);
    }

    // 4. Enviar gripper
    if (!pose.gripper_joints.empty()) {
      send_trajectory(
        gripper_client_,
        {"gripper_left_finger_joint", "gripper_right_finger_joint"},
        pose.gripper_joints, 1);
    }
  }

  void send_trajectory(
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    int sec)
  {
    if (!client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Action server no disponible para articulaciones solicitadas");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start.sec = sec;
    goal_msg.trajectory.points.push_back(point);

    client->async_send_goal(goal_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}