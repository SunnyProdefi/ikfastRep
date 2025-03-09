#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <array>
#include <string>

enum JointID : int
{
    JOINT1_1,
    JOINT1_2,
    JOINT1_3,
    JOINT1_4,
    JOINT1_5,
    JOINT1_6,
    JOINT_COUNT
};

// 将枚举 ID 与具体关节名称一一对应
static const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {
    "Joint1_1", "Joint1_2", "Joint1_3", "Joint1_4", "Joint1_5", "Joint1_6",
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "floating_joint_control_node");
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

    // 设置关节初始位置
    joint_msg.position[JOINT1_1] = -0.00402422;
    joint_msg.position[JOINT1_2] = 0.243699;
    joint_msg.position[JOINT1_3] = -0.0481361;
    joint_msg.position[JOINT1_4] = 1.09856;
    joint_msg.position[JOINT1_5] = -0.428438;
    joint_msg.position[JOINT1_6] = 0.712843;

    // 创建 TF 变换广播器
    static tf::TransformBroadcaster br;

    // 可以将恒定坐标与姿态放在循环外部，避免重复赋值
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.402303, -0.057641, 0.737771));

    // // 这里使用一个单位四元数，表示无旋转
    // tf::Quaternion q;
    // q.setRPY(0.0, 0.0, 0.0);
    // transform.setRotation(q);

    // 这里使用一个单位四元数，表示无旋转
    tf::Quaternion q;
    q.setX(0.433333);
    q.setY(0.0118536);
    q.setZ(0.00467116);
    q.setW(0.901144);
    transform.setRotation(q);

    ros::Rate rate(50);  // 50Hz

    while (ros::ok())
    {
        // 获取当前时间戳
        ros::Time current_time = ros::Time::now();

        // 填充并发布 TF 变换消息
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

        // 发布 JointState 消息
        joint_msg.header.stamp = current_time;
        joint_pub.publish(joint_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
