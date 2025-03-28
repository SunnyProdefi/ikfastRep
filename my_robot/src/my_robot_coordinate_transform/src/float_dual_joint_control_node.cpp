#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <vector>
#include <string>
#include <fstream>

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

void loadJointData(const std::string& filename, std::vector<double>& init_positions, std::vector<double>& goal_positions, tf::Vector3& init_translation, tf::Quaternion& init_rotation, tf::Vector3& target_translation, tf::Quaternion& target_rotation)
{
    std::string yaml_path = ros::package::getPath("my_robot_coordinate_transform") + "/config/joint_angle.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (config["init_joint_angles"] && config["joint_angles"])
    {
        init_positions = config["init_joint_angles"].as<std::vector<double>>();
        goal_positions = config["joint_angles"].as<std::vector<double>>();
    }
    if (config["init_floating_base"] && config["floating_base"])
    {
        std::vector<double> init_fb = config["init_floating_base"].as<std::vector<double>>();
        std::vector<double> goal_fb = config["floating_base"].as<std::vector<double>>();

        init_translation = tf::Vector3(init_fb[0], init_fb[1], init_fb[2]);
        init_rotation = tf::Quaternion(init_fb[3], init_fb[4], init_fb[5], init_fb[6]);
        init_rotation.normalize();

        target_translation = tf::Vector3(goal_fb[0], goal_fb[1], goal_fb[2]);
        target_rotation = tf::Quaternion(goal_fb[3], goal_fb[4], goal_fb[5], goal_fb[6]);
        target_rotation.normalize();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "float_dual_joint_control_node");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    std::vector<double> init_positions, goal_positions;
    tf::Vector3 init_translation, target_translation;
    tf::Quaternion init_rotation, target_rotation;

    loadJointData("../config/joint_angle.yaml", init_positions, goal_positions, init_translation, init_rotation, target_translation, target_rotation);

    sensor_msgs::JointState joint_msg;
    joint_msg.name.assign(JOINT_NAMES.begin(), JOINT_NAMES.end());
    joint_msg.position.assign(init_positions.begin(), init_positions.end());

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(init_translation);
    transform.setRotation(init_rotation);

    ros::Rate rate(50);
    double alpha = 0.02;
    double time_elapsed = 0.0;
    double switch_time = 10.0;  // 每 5 秒切换方向
    bool moving_to_goal = true;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        time_elapsed += rate.expectedCycleTime().toSec();
        std::cout << "time_elapsed: " << time_elapsed << std::endl;

        if (time_elapsed >= switch_time)
        {
            moving_to_goal = !moving_to_goal;
            time_elapsed = 0.0;
            ROS_INFO_STREAM("Switching direction: " << (moving_to_goal ? "Moving to goal" : "Returning to init"));
        }

        std::vector<double>& current_target = moving_to_goal ? goal_positions : init_positions;
        for (int i = 0; i < JOINT_COUNT; ++i)
        {
            joint_msg.position[i] = (1 - alpha) * joint_msg.position[i] + alpha * current_target[i];
        }

        tf::Vector3& tf_target_translation = moving_to_goal ? target_translation : init_translation;
        tf::Quaternion& tf_target_rotation = moving_to_goal ? target_rotation : init_rotation;

        tf::Vector3 new_translation = (1 - alpha) * transform.getOrigin() + alpha * tf_target_translation;
        tf::Quaternion new_rotation = transform.getRotation().slerp(tf_target_rotation, alpha);

        transform.setOrigin(new_translation);
        transform.setRotation(new_rotation);

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

        joint_msg.header.stamp = current_time;
        joint_pub.publish(joint_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}