#include <memory>
#include <iostream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
    }

  void init_move_group() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface");

    auto initial_pose = move_group_interface_->getCurrentPose();

    RCLCPP_INFO(this->get_logger(), "Initial position: x=%.3f, y=%.3f, z=%.3f", 
                initial_pose.pose.position.x, 
                initial_pose.pose.position.y, 
                initial_pose.pose.position.z);

    RCLCPP_INFO(this->get_logger(), "Initial orientation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                initial_pose.pose.orientation.x,
                initial_pose.pose.orientation.y,
                initial_pose.pose.orientation.z,
                initial_pose.pose.orientation.w);

    //RCLCPP_INFO(this->get_logger(), "Initalizing PlanningSceneMonitor");
    //planning_scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");

    //const planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();

    RCLCPP_INFO(this->get_logger(), "Attaching camera tool to robot model");

    moveit_msgs::msg::AttachedCollisionObject object_to_attach;
    object_to_attach.object.id = "camera_tool";

    shape_msgs::msg::SolidPrimitive rectangle;
    rectangle.type = shape_msgs::msg::SolidPrimitive::BOX;
    rectangle.dimensions.resize(3);
    rectangle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.1;
    rectangle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.1;
    rectangle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.1;

    object_to_attach.object.header.frame_id = "tool0";
    object_to_attach.object.primitives.push_back(rectangle);
    object_to_attach.object.operation = object_to_attach.object.ADD;

    object_to_attach.link_name = object_to_attach.object.header.frame_id;
    object_to_attach.touch_links.push_back("wrist_2_link");
    object_to_attach.touch_links.push_back("wrist_3_link");

    planning_scene_interface_.applyAttachedCollisionObject(object_to_attach);
  }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    //std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

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

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<JointTargetSubscriberNode>();
  
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   node->init_move_group();

//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTargetSubscriberNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  node->init_move_group();

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}


