#include <iostream>
#include <vector>
#include <cmath>                          // 用于误差计算
#include <limits>                         // 用于 std::numeric_limits
#include "ikfast_wrapper_single_arm.cpp"  // 替换为包含 Kinematics 类声明的头文件名

struct JointLimit
{
    float lower;
    float upper;
};

const std::vector<JointLimit> joint_limits = {
    {-2.3562f, 4.7124f},  // Joint2_1
    {-2.0944f, 1.5708f},  // Joint2_2
    {-2.5307f, 2.5307f},  // Joint2_3
    {-3.1416f, 3.1416f},  // Joint2_4
    {-1.9199f, 1.9199f},  // Joint2_5
    {-3.1416f, 3.1416f}   // Joint2_6
};

bool isSolutionWithinLimits(const std::vector<float>& solution, const std::vector<JointLimit>& limits)
{
    if (solution.size() != limits.size())
        return false;

    for (size_t i = 0; i < solution.size(); ++i)
    {
        if (solution[i] < limits[i].lower || solution[i] > limits[i].upper)
        {
            return false;
        }
    }
    return true;
}

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

    std::vector<float> joint_config = {-0.488, 0.171, 2.179, -1.596, -1.238, -0.312};

    std::vector<float> ee_pose = {-0.92778, 0.2269, 0.3023, 0.419, 0.13895, 0.94801, -0.28651, -0.447, -0.35156, -0.22334, -0.91113, -0.096};

    // 分支2
    // std::vector<float> ee_pose = {-5.836303e-07, 1.673404e-08, 1, 0.48, -0.7071064, 0.7071072, -4.245215e-07, -0.43, -0.7071072, -0.7071064, -4.008564e-07, 0.30};
    // 分支3
    // 加前减后、加左减右、加上减下
    // std::vector<float> ee_pose = {-5.836303e-07, -1.673424e-08, 1, 0.48, -0.7071061, -0.7071075, -4.245215e-07, -0.43, 0.7071075, -0.7071061, 4.008564e-07, -0.30};

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

            // 检查关节是否在限制范围内
            if (!isSolutionWithinLimits(solution, joint_limits))
            {
                std::cout << "Skipped invalid solution (out of joint limits): ";
                for (float val : solution) std::cout << val << ", ";
                std::cout << std::endl;
                continue;
            }

            float distance = calculateDistance(joint_config, solution);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_solution = solution;
            }

            std::cout << "Valid solution: ";
            for (size_t j = 0; j < solution.size(); ++j)
            {
                std::cout << solution[j] << ", ";
            }
            std::cout << "(Distance: " << distance << ")" << std::endl;
        }

        // 输出最接近的解
        if (!closest_solution.empty())
        {
            std::cout << "Closest Solution to the Original Joint Configuration:" << std::endl;
            for (size_t i = 0; i < closest_solution.size(); ++i)
            {
                std::cout << closest_solution[i] << ", ";
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
