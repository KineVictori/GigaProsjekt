#include <moveit/move_group_interface/move_group_interface.h>

void setHomePosition() {
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm_group");
    std::vector<double> home_joints = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
    
    move_group->setJointValueTarget(home_joints);
    move_group->setStartStateToCurrentState();
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group->execute(plan);
    }
}
