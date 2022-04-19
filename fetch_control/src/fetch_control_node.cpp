#include <ros/ros.h>
#include <ros/console.h>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fetch_controller");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    moveit::planning_interface::MoveGroupInterface move_group("arm_with_torso");
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    geometry_msgs::Pose target_pose,target_pose_1;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success;
    while (ros::ok())
    {
        ROS_INFO_STREAM("Reference frame: " << move_group.getEndEffectorLink().c_str());
        move_group.setMaxVelocityScalingFactor(1.0);
        // Move to pose
        // target_pose.orientation.x = 0.0;
        // target_pose.orientation.y = 0.0;
        // target_pose.orientation.z = 0.0;
        // target_pose.orientation.w = 1.0;
        // target_pose.position.x = 0.28;
        // target_pose.position.y = -0.7;
        // target_pose.position.z = 1.0;
        // move_group.setPoseTarget(target_pose);
        // ROS_INFO_STREAM("Set_pose");
        
        // move_group.move();
        // ROS_INFO_STREAM("Move:");


        // Move to joint
        std::vector<double> joint_values;
        ROS_INFO_STREAM("Reference frame:" << move_group.getCurrentState());

        joint_values[0] = 0.0;
        joint_values[1] = 1.5;
        joint_values[2] = -0.6;
        joint_values[3] = 3.0;
        joint_values[4] = 1.0;
        joint_values[5] = 3.0;
        joint_values[6] = 1.0;
        // joint_values[7] = 3.0;
        move_group.setJointValueTarget(joint_values);
        ROS_INFO_STREAM("Set_joint");

        move_group.move();
        ROS_INFO_STREAM("Move:");

        //rate
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}