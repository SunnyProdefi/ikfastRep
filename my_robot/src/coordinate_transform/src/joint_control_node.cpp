#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <array>
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

// 将枚举 ID 与具体关节名称一一对应
static const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {"Joint2_1", "Joint2_2", "Joint2_3", "Joint2_4", "Joint2_5", "Joint2_6", "Joint3_1", "Joint3_2", "Joint3_3", "Joint3_4", "Joint3_5", "Joint3_6"};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_control_node");
    ros::NodeHandle nh;

    // 创建 JointState 发布器
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 准备 JointState 消息
    sensor_msgs::JointState joint_msg;
    joint_msg.name.resize(JOINT_COUNT);
    joint_msg.position.resize(JOINT_COUNT);

    // 将关节名拷贝到消息中
    for (int i = 0; i < JOINT_COUNT; ++i)
    {
        joint_msg.name[i] = JOINT_NAMES[i];
    }

    // // 设置关节初始位置
    // joint_msg.position[JOINT2_1] = -30.0 * M_PI / 180.0;
    // joint_msg.position[JOINT2_2] = -45.0 * M_PI / 180.0;
    // joint_msg.position[JOINT2_3] = -90.0 * M_PI / 180.0;
    // joint_msg.position[JOINT2_4] = -45.0 * M_PI / 180.0;
    // joint_msg.position[JOINT2_5] = -60 * M_PI / 180.0;
    // joint_msg.position[JOINT2_6] = -15.0 * M_PI / 180.0;

    // joint_msg.position[JOINT3_1] = 30.0 * M_PI / 180.0;
    // joint_msg.position[JOINT3_2] = -45.0 * M_PI / 180.0;
    // joint_msg.position[JOINT3_3] = -90.0 * M_PI / 180.0;
    // joint_msg.position[JOINT3_4] = 45.0 * M_PI / 180.0;
    // joint_msg.position[JOINT3_5] = -60.0 * M_PI / 180.0;
    // joint_msg.position[JOINT3_6] = 15.0 * M_PI / 180.0;

    // 设置关节末端位置
    joint_msg.position[JOINT2_1] = -30.0 * M_PI / 180.0;
    joint_msg.position[JOINT2_2] = -45.0 * M_PI / 180.0;
    joint_msg.position[JOINT2_3] = -90.0 * M_PI / 180.0;
    joint_msg.position[JOINT2_4] = -45.0 * M_PI / 180.0;
    joint_msg.position[JOINT2_5] = 0 * M_PI / 180.0;
    joint_msg.position[JOINT2_6] = -15.0 * M_PI / 180.0;

    joint_msg.position[JOINT3_1] = 30.0 * M_PI / 180.0;
    joint_msg.position[JOINT3_2] = -45.0 * M_PI / 180.0;
    joint_msg.position[JOINT3_3] = -90.0 * M_PI / 180.0;
    joint_msg.position[JOINT3_4] = 45.0 * M_PI / 180.0;
    joint_msg.position[JOINT3_5] = 0 * M_PI / 180.0;
    joint_msg.position[JOINT3_6] = 15.0 * M_PI / 180.0;

    ros::Rate rate(50);  // 50Hz

    while (ros::ok())
    {
        // 获取当前时间戳
        ros::Time current_time = ros::Time::now();

        // 发布 JointState 消息
        joint_msg.header.stamp = current_time;
        joint_pub.publish(joint_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
