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

    std::vector<float> joint_config = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // 测试已知的末端执行器旋转矩阵和位移
    // Link1_0 -> dummy_point1 :
    // std::vector<float> ee_pose = {// Row 1
    //                               -1.0f, 9.00563e-05f, 1.0752e-06f, 0.452049f,
    //                               // Row 2
    //                               -6.44401e-05f, -0.707112f, -0.707102f, -0.626356f,
    //                               // Row 3
    //                               -6.29186e-05f, -0.707102f, 0.707112f, -0.0212478f};

    // Link4_0 -> dummy_point2 :
    // std::vector<float> ee_pose = {-1,           9.52509e-05, 1.07516e-06, 0.45195,    //
    //                               6.65918e-05,  0.707102,    -0.707112,   -0.626284,  //
    //                               -6.81133e-05, -0.707112,   -0.707102,   0.021181};

    // Link2_0 -> dummy_point3 :
    std::vector<float> ee_pose = {-1, 9.00563e-05, 6.26989e-06, 0.45204, -5.92454e-05, -0.707102, 0.707112, 0.172674, 6.81133e-05, 0.707112, 0.707102, 0.636431};

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
