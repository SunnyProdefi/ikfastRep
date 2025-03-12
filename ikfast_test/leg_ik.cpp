#include <iostream>
#include <vector>
#include <cmath>                          // 误差计算
#include <limits>                         // 用于 std::numeric_limits
#include <yaml-cpp/yaml.h>                // 读取 YAML 文件
#include "ikfast_wrapper_single_arm.cpp"  // 替换为包含 Kinematics 类声明的头文件名
#include <fstream>                        // 用于写入 YAML 文件

// **从 YAML 读取 ee_pose（转换为 4x4 矩阵格式）**
std::vector<float> loadEEPoseFromYAML(const std::string& filename, const std::string& tf_name)
{
    YAML::Node yaml_data;
    try
    {
        yaml_data = YAML::LoadFile(filename);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return {};
    }

    if (!yaml_data[tf_name])
    {
        std::cerr << "Error: " << tf_name << " not found in YAML file!" << std::endl;
        return {};
    }

    YAML::Node pose_data = yaml_data[tf_name]["target_pose"];
    if (!pose_data)
    {
        std::cerr << "Error: target_pose not found!" << std::endl;
        return {};
    }

    // **解析 Position**
    std::vector<float> position;
    for (int i = 0; i < 3; ++i)
    {
        position.push_back(pose_data["position"][i].as<float>());
    }

    // **解析 Orientation (3x3 旋转矩阵)**
    std::vector<float> rotation_matrix;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rotation_matrix.push_back(pose_data["orientation"][i][j].as<float>());
        }
    }

    // **构造 ee_pose (3x4 矩阵，按行存储)**
    std::vector<float> ee_pose;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            ee_pose.push_back(rotation_matrix[row * 3 + col]);  // 旋转矩阵部分
        }
        ee_pose.push_back(position[row]);  // 位置部分
    }

    return ee_pose;
}

// **计算欧几里得距离**
float calculateDistance(const std::vector<float>& a, const std::vector<float>& b)
{
    float distance = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
    {
        distance += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(distance);
}

// **计算所有解，并找到最优解**
std::pair<std::vector<std::vector<float>>, std::vector<float>> findAllSolutions(const std::vector<float>& ik_results, const std::vector<float>& initial_q, size_t num_joints)
{
    std::vector<std::vector<float>> all_solutions;
    std::vector<float> closest_solution;
    float min_distance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < ik_results.size(); i += num_joints)
    {
        std::vector<float> solution(ik_results.begin() + i, ik_results.begin() + i + num_joints);
        all_solutions.push_back(solution);

        float distance = calculateDistance(initial_q, solution);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_solution = solution;
        }
    }
    return {all_solutions, closest_solution};
}

// **保存计算结果到 YAML**
void saveResultToYAML(const std::string& filename, const std::vector<std::vector<float>>& left_solutions, const std::vector<float>& best_left, const std::vector<std::vector<float>>& right_solutions, const std::vector<float>& best_right)
{
    YAML::Node result;

    // **存储左腿所有解**
    YAML::Node left_leg_solutions;
    for (const auto& sol : left_solutions)
    {
        YAML::Node joint_set;
        for (float value : sol)
        {
            joint_set.push_back(value);
        }
        joint_set.SetStyle(YAML::EmitterStyle::Flow);  // **单个解保持 Flow**
        left_leg_solutions.push_back(joint_set);
    }
    result["left_leg_solutions"] = left_leg_solutions;  // **不设置 Flow，确保换行**

    // **存储最优解**
    YAML::Node best_left_leg;
    for (float value : best_left)
    {
        best_left_leg.push_back(value);
    }
    best_left_leg.SetStyle(YAML::EmitterStyle::Flow);  // **保持 Flow 格式**
    result["best_left_leg_solution"] = best_left_leg;

    // **存储右腿所有解**
    YAML::Node right_leg_solutions;
    for (const auto& sol : right_solutions)
    {
        YAML::Node joint_set;
        for (float value : sol)
        {
            joint_set.push_back(value);
        }
        joint_set.SetStyle(YAML::EmitterStyle::Flow);  // **单个解保持 Flow**
        right_leg_solutions.push_back(joint_set);
    }
    result["right_leg_solutions"] = right_leg_solutions;  // **不设置 Flow，确保换行**

    // **存储最优解**
    YAML::Node best_right_leg;
    for (float value : best_right)
    {
        best_right_leg.push_back(value);
    }
    best_right_leg.SetStyle(YAML::EmitterStyle::Flow);  // **保持 Flow 格式**
    result["best_right_leg_solution"] = best_right_leg;

    // **写入 YAML 文件**
    std::ofstream fout(filename);
    if (fout.is_open())
    {
        fout << result;
        fout.close();
        std::cout << "IK results saved to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
    }
}

int main()
{
    // **从 YAML 读取 ee_pose**
    std::string yaml_file = "../../my_robot/src/coordinate_transform/config/leg_ik.yaml";
    std::string result_path = "../../my_robot/src/coordinate_transform/config/result.yaml";
    std::string tf_mat_link1_0_flan1 = "tf_mat_link1_0_flan1";
    std::string tf_mat_link4_0_flan4 = "tf_mat_link4_0_flan4";
    std::vector<float> ee_pose_l = loadEEPoseFromYAML(yaml_file, tf_mat_link1_0_flan1);
    std::vector<float> ee_pose_r = loadEEPoseFromYAML(yaml_file, tf_mat_link4_0_flan4);

    if (ee_pose_l.empty() || ee_pose_r.empty())
    {
        std::cerr << "Failed to load ee_pose from YAML file." << std::endl;
        return -1;
    }

    // **创建 Kinematics 对象**
    robots::Kinematics kinematics;

    // **初始化关节角**
    std::vector<float> leg_l_q_init = {1.597727, 0.295055, 2.156446, 0.040097, 0.494959, 3.125349};
    std::vector<float> leg_r_q_init = {-1.597727, 0.295055, 2.156446, -0.040097, 0.494959, 0.016244};

    // **计算逆运动学解**
    std::vector<float> leg_l_q_res = kinematics.inverse(ee_pose_l);
    std::vector<float> leg_r_q_res = kinematics.inverse(ee_pose_r);

    // **计算所有解并找到最优解**
    auto [left_solutions, best_left] = findAllSolutions(leg_l_q_res, leg_l_q_init, kinematics.num_of_joints);
    auto [right_solutions, best_right] = findAllSolutions(leg_r_q_res, leg_r_q_init, kinematics.num_of_joints);

    // **保存结果**
    saveResultToYAML(result_path, left_solutions, best_left, right_solutions, best_right);

    return 0;
}
