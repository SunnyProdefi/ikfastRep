#include <iostream>
#include <vector>
#include <cmath>                          // 用于误差计算
#include <limits>                         // 用于 std::numeric_limits
#include "ikfast_wrapper_single_arm.cpp"  // 替换为包含 Kinematics 类声明的头文件名

float calculateDistance(const std::vector<float>& a, const std::vector<float>& b)
{
    float distance = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
    {
        distance += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(distance);
}

int main()
{
    // 创建一个 Kinematics 对象
    robots::Kinematics kinematics;

    // 测试关节角度配置（假设有 6 个关节）
    std::vector<float> joint_config = {1.76782, -2.0944, 1.89509, -3.1416, -0.278314, -3.1416};
    std::cout << "Input Joint Configurations:" << std::endl;
    for (size_t i = 0; i < joint_config.size(); ++i)
    {
        std::cout << joint_config[i] << " ";
    }
    std::cout << std::endl;

    // 调用正向运动学函数
    std::vector<float> ee_pose = kinematics.forward(joint_config);
    if (!ee_pose.empty())
    {
        std::cout << "Forward Kinematics Result (End Effector Pose):" << std::endl;
        for (size_t i = 0; i < ee_pose.size(); ++i)
        {
            std::cout << ee_pose[i] << " ";
            if ((i + 1) % 4 == 0)
                std::cout << std::endl;  // 每行输出4个值
        }
    }
    else
    {
        std::cerr << "Forward kinematics failed due to invalid input." << std::endl;
        return -1;
    }

    // 使用正向运动学输出调用逆向运动学函数
    std::vector<float> joint_configs = kinematics.inverse(ee_pose);
    if (!joint_configs.empty())
    {
        std::cout << "Inverse Kinematics Solutions (Joint Configurations):" << std::endl;
        size_t num_joints = kinematics.num_of_joints;
        std::vector<float> closest_solution;
        float min_distance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < joint_configs.size(); i += num_joints)
        {
            std::vector<float> solution(joint_configs.begin() + i, joint_configs.begin() + i + num_joints);
            // 计算与原始 joint_config 的距离
            float distance = calculateDistance(joint_config, solution);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_solution = solution;
            }

            // 输出当前解
            for (size_t j = 0; j < solution.size(); ++j)
            {
                std::cout << solution[j] << " ";
            }
            std::cout << "(Distance: " << distance << ")" << std::endl;
        }

        // 输出最接近的解
        if (!closest_solution.empty())
        {
            std::cout << "Closest Solution to the Original Joint Configuration:" << std::endl;
            for (size_t i = 0; i < closest_solution.size(); ++i)
            {
                std::cout << closest_solution[i] << " ";
            }
            std::cout << std::endl << "Minimum Distance: " << min_distance << std::endl;
        }
    }
    else
    {
        std::cerr << "Inverse kinematics failed to find a solution." << std::endl;
    }

    return 0;
}
