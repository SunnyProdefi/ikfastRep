#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <array>
#include <vector>
#include <string>

enum JointID : int
{
    JOINT2_1,
    JOINT2_2,
    JOINT2_3,
    JOINT2_4,
    JOINT2_5,
    JOINT2_6,
    JOINT3_1,
    JOINT3_2,
    JOINT3_3,
    JOINT3_4,
    JOINT3_5,
    JOINT3_6,
    JOINT_COUNT
};

static const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {"Joint2_1", "Joint2_2", "Joint2_3", "Joint2_4", "Joint2_5", "Joint2_6", "Joint3_1", "Joint3_2", "Joint3_3", "Joint3_4", "Joint3_5", "Joint3_6"};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "float_dual_joint_control_node");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    sensor_msgs::JointState joint_msg;
    joint_msg.name.resize(JOINT_COUNT);
    joint_msg.position.resize(JOINT_COUNT);
    for (int i = 0; i < JOINT_COUNT; ++i)
    {
        joint_msg.name[i] = JOINT_NAMES[i];
    }

    // 初始关节位姿
    std::array<double, JOINT_COUNT> init_positions = {-0.5236, -0.7854, -1.5708, -0.7854, -1.0472, -0.2618, 0.5236, -0.7854, -1.5708, 0.7854, -1.0472, 0.2618};

    // 目标关节位姿
    std::array<double, JOINT_COUNT> goal_positions = {-0.356351, -0.816277, -1.37537, -0.8114, -0.0857247, -0.2862287, 0.356607, -0.816145, -1.37504, 0.812594, -0.0869601, 0.285634};

    joint_msg.position.assign(init_positions.begin(), init_positions.end());

    std::vector<double> init_positions_vec(init_positions.begin(), init_positions.end());
    std::vector<double> goal_positions_vec(goal_positions.begin(), goal_positions.end());

    // TF 变换
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0.5));

    tf::Quaternion q(0, 0.7071, 0, 0.7071);
    q.normalize();
    transform.setRotation(q);

    tf::Vector3 init_translation(0, 0, 0.5);
    tf::Quaternion init_rotation = q;

    tf::Vector3 target_translation(0.430362, 0, 0.382518);
    tf::Quaternion target_rotation(-0.000166733, 0.792409, -0.000239329, 0.609991);
    target_rotation.normalize();

    ros::Rate rate(50);
    double alpha = 0.02;
    bool moving_to_goal = true;  // 方向标志

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        // 关节插值
        std::vector<double>& current_target = moving_to_goal ? goal_positions_vec : init_positions_vec;
        bool joint_reached = true;

        for (int i = 0; i < JOINT_COUNT; ++i)
        {
            double new_pos = (1 - alpha) * joint_msg.position[i] + alpha * current_target[i];
            joint_msg.position[i] = new_pos;

            if (fabs(new_pos - current_target[i]) > 0.01)  // 允许的误差
                joint_reached = false;
        }

        // TF 变换插值
        tf::Vector3& tf_target_translation = moving_to_goal ? target_translation : init_translation;
        tf::Quaternion& tf_target_rotation = moving_to_goal ? target_rotation : init_rotation;

        tf::Vector3 new_translation = (1 - alpha) * transform.getOrigin() + alpha * tf_target_translation;
        tf::Quaternion new_rotation = transform.getRotation().slerp(tf_target_rotation, alpha);

        transform.setOrigin(new_translation);
        transform.setRotation(new_rotation);

        // 检测 TF 是否到达目标
        bool tf_reached = (transform.getOrigin() - tf_target_translation).length() < 0.01 && transform.getRotation().angle(tf_target_rotation) < 0.01;

        // 如果关节和TF都到达目标位置，则反转方向
        if (joint_reached && tf_reached)
        {
            moving_to_goal = !moving_to_goal;
            ROS_INFO_STREAM("Switching direction: " << (moving_to_goal ? "Moving to goal" : "Returning to init"));
        }

        // 发布 TF
        geometry_msgs::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "world";
        t.child_frame_id = "base_link";
        t.transform.translation.x = transform.getOrigin().x();
        t.transform.translation.y = transform.getOrigin().y();
        t.transform.translation.z = transform.getOrigin().z();
        t.transform.rotation.x = transform.getRotation().x();
        t.transform.rotation.y = transform.getRotation().y();
        t.transform.rotation.z = transform.getRotation().z();
        t.transform.rotation.w = transform.getRotation().w();
        br.sendTransform(t);

        // 发布 JointState
        joint_msg.header.stamp = current_time;
        joint_pub.publish(joint_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
