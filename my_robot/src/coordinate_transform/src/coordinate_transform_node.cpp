#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_transform_node");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            // 获取 world 到 flan 的变换
            listener.lookupTransform("world", "Link2_6", ros::Time(0), transform);

            // 打印变换的平移部分和旋转部分
            tf::Matrix3x3 rotation_matrix(transform.getRotation());

            // 创建 Eigen 的 3x3 旋转矩阵
            Eigen::Matrix3f eigen_rotation_matrix;

            // 手动提取 tf::Matrix3x3 中的旋转矩阵数据并赋值给 Eigen::Matrix3f
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    eigen_rotation_matrix(i, j) = rotation_matrix[i][j];
                }
            }

            // 创建 Eigen 的 4x4 变换矩阵
            Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

            // 设置旋转部分
            transform_matrix.block<3, 3>(0, 0) = eigen_rotation_matrix;

            // 设置平移部分
            transform_matrix(0, 3) = transform.getOrigin().x();
            transform_matrix(1, 3) = transform.getOrigin().y();
            transform_matrix(2, 3) = transform.getOrigin().z();

            // 输出4x4矩阵
            ROS_INFO_STREAM("Transformation Matrix (world -> Link2_6):");
            std::cout << transform_matrix << std::endl;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
