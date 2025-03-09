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

    // 设置关节初始位置
    joint_msg.position[JOINT2_1] = -0.422489;
    joint_msg.position[JOINT2_2] = 0.309904;
    joint_msg.position[JOINT2_3] = -0.0735355;
    joint_msg.position[JOINT2_4] = -0.0435693;
    joint_msg.position[JOINT2_5] = -0.0224282;
    joint_msg.position[JOINT2_6] = 0.258911;

    joint_msg.position[JOINT3_1] = 0.38897;
    joint_msg.position[JOINT3_2] = 0.393305;
    joint_msg.position[JOINT3_3] = -0.163173;
    joint_msg.position[JOINT3_4] = 0.272361;
    joint_msg.position[JOINT3_5] = 0.177097;
    joint_msg.position[JOINT3_6] = -0.209306;

    // 创建 TF 变换广播器
    static tf::TransformBroadcaster br;

    // 可以将恒定坐标与姿态放在循环外部，避免重复赋值
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.115411, -0.0814905, 0.437057));

    // // 这里使用一个单位四元数，表示无旋转
    // tf::Quaternion q;
    // q.setRPY(0.0, 0.0, 0.0);
    // transform.setRotation(q);

    // 这里使用一个单位四元数，表示无旋转
    tf::Quaternion q;
    q.setX(-0.0606923);
    q.setY(0.476322);
    q.setZ(-0.025329);
    q.setW(0.876808);
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
