#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "floating_joint_control_node");
    ros::NodeHandle nh;

    // 创建 TF 变换广播器
    static tf::TransformBroadcaster br;

    ros::Rate rate(50);  // 50Hz
    double start_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        double current_time = ros::Time::now().toSec();
        double elapsed = current_time - start_time;

        // 简单演示：让 base_link 在 XY 平面上做半径为 1 的匀速圆周运动，并绕 Z 轴旋转
        double radius = 1.0;
        double angular_speed = 0.5;  // rad/s
        double x = radius * std::cos(angular_speed * elapsed);
        double y = radius * std::sin(angular_speed * elapsed);
        double z = 0.0;

        // 设置绕 Z 轴的旋转角度
        double yaw = angular_speed * elapsed;
        double pitch = 0.0;
        double roll = 0.0;

        // 使用 TF 的方法，将欧拉角(RPY)转换为四元数
        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);

        // 构造并发布 TF 变换消息
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "world";     // 父坐标系
        t.child_frame_id = "base_link";  // 子坐标系

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        br.sendTransform(t);

        rate.sleep();
    }

    return 0;
}
