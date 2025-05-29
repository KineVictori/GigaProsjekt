#include <memory>
#include <iostream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>

class JointTargetSubscriberNode : public rclcpp::Node {
public:
  JointTargetSubscriberNode()
    : Node("joint_position_mover", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", false));
      

      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/target_robot_pose", 10,
        std::bind(&JointTargetSubscriberNode::joint_callback, this, std::placeholders::_1)
      );

      task_complete_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/task_complete", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Task complete received, returning to home.");
            if (move_group_interface_) {
                set_home_position();
            } else {
                RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized! Cannot return to home.");
            }
        }
    );

      // Home command subscription
      go_home_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/go_home", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Go home command received.");
          if (move_group_interface_) {
            set_home_position();
          } else {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized! Cannot return to home.");
          }
        }
      );
    }

  void init_move_group() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface");

    RCLCPP_INFO(this->get_logger(), "Attaching camera tool to robot model");

    moveit_msgs::msg::AttachedCollisionObject camera_tool_object;
    camera_tool_object.object.id = "camera_tool";
    camera_tool_object.object.header.frame_id = "tool0";

    // Adding a box that simulates being the camera, to prevent bumping the camera
    shape_msgs::msg::SolidPrimitive camera_tool_box;
    camera_tool_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    camera_tool_box.dimensions.resize(3);
    camera_tool_box.dimensions[camera_tool_box.BOX_X] = 0.1;
    camera_tool_box.dimensions[camera_tool_box.BOX_Y] = 0.1;
    camera_tool_box.dimensions[camera_tool_box.BOX_Z] = 0.1;

    camera_tool_object.object.primitives.push_back(camera_tool_box);
    camera_tool_object.object.operation = camera_tool_object.object.ADD;

    camera_tool_object.link_name = camera_tool_object.object.header.frame_id;
    camera_tool_object.touch_links.push_back("wrist_2_link");
    camera_tool_object.touch_links.push_back("wrist_3_link");

    planning_scene_interface_.applyAttachedCollisionObject(camera_tool_object);
    move_group_interface_->attachObject("camera_tool", "tool0");

    // Add in a ground object, such that robot knows that it cannot move there
    moveit_msgs::msg::CollisionObject ground_object;
    ground_object.id = "ground";
    ground_object.header.frame_id = move_group_interface_->getPlanningFrame();

    shape_msgs::msg::SolidPrimitive ground_primitive;
    ground_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    ground_primitive.dimensions.resize(3);
    ground_primitive.dimensions[ground_primitive.BOX_X] = 1.0;
    ground_primitive.dimensions[ground_primitive.BOX_Y] = 0.75;
    ground_primitive.dimensions[ground_primitive.BOX_Z] = 0.02;

    geometry_msgs::msg::Pose ground_pose;
    ground_pose.position.x = ground_primitive.dimensions[ground_primitive.BOX_X] * 0.4;
    ground_pose.position.y = 0;
    ground_pose.position.z = 0;

    ground_object.primitives.push_back(ground_primitive);
    ground_object.primitive_poses.push_back(ground_pose);
    ground_object.operation = ground_object.ADD;

    planning_scene_interface_.applyCollisionObject(ground_object);
    move_group_interface_->attachObject("ground", "base_link_inertia");
    
    // move to home at start
    set_home_position();
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized and home position set.");
  }

  void set_home_position() {
    std::vector<double> home_joints = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0}; // {shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3}

    move_group_interface_->setJointValueTarget(home_joints);
    move_group_interface_->setStartStateToCurrentState();

    // create a plan to move to the home position
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(this->get_logger(), "Setting home position...");
    
    // if successfully planned, execute the plan
    if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_interface_->execute(plan);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set home position.");
    }
  // RCLCPP_INFO(this->get_logger(), "Home position set successfully.");
  }


private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_complete_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr go_home_subscription_;

    void joint_callback(const geometry_msgs::msg::Pose &pose) {
      move_group_interface_->setPoseTarget(pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_interface_->plan(plan));

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
        move_group_interface_->execute(plan);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      }
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTargetSubscriberNode>();
  node->init_move_group();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



