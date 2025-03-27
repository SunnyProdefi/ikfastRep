#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <ros/package.h>

// **加载 YAML 文件**
std::string package_path = ros::package::getPath("coordinate_transform");
std::string floating_base_file = package_path + "/config/floating_base.yaml";
std::string tf_using_file = package_path + "/config/tf_using.yaml";
std::string output_file = package_path + "/config/leg_ik_cs.yaml";
// **读取 YAML 文件**
YAML::Node loadYAML(const std::string& filename)
{
    try
    {
        return YAML::LoadFile(filename);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to load YAML file: " << filename << ". Error: " << e.what());
        return YAML::Node();
    }
}

// **解析四元数为旋转矩阵**
Eigen::Matrix3f quaternionToRotationMatrix(float x, float y, float z, float w)
{
    Eigen::Quaternionf q(w, x, y, z);
    return q.toRotationMatrix();
}

// **解析 position + orientation 变换矩阵**
Eigen::Matrix4f parseTransformMatrix(const YAML::Node& node)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    if (!node)
    {
        ROS_ERROR("Invalid transform node.");
        return transform;
    }
    // 读取 position
    Eigen::Vector3f position(node["target_pose"]["position"][0].as<float>(), node["target_pose"]["position"][1].as<float>(), node["target_pose"]["position"][2].as<float>());

    // 读取 orientation
    Eigen::Matrix3f rotation;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rotation(i, j) = node["target_pose"]["orientation"][i][j].as<float>();
        }
    }

    // 组合到 4x4 变换矩阵
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = position;
    return transform;
}

// **保存计算结果到 leg_ik.yaml**
void saveToYAML(const std::string& filename, const Eigen::Matrix4f& tf_link1_0_flan1, const Eigen::Matrix4f& tf_link4_0_flan4)
{
    YAML::Node root;

    // 处理两个转换矩阵
    for (int i = 0; i < 2; i++)
    {
        Eigen::Matrix4f tf = (i == 0) ? tf_link1_0_flan1 : tf_link4_0_flan4;
        std::string name = (i == 0) ? "tf_mat_link1_0_flan1" : "tf_mat_link4_0_flan4";

        YAML::Node tf_data;

        // **修正 Position 存储**
        YAML::Node position;
        position.push_back(tf(0, 3));
        position.push_back(tf(1, 3));
        position.push_back(tf(2, 3));
        position.SetStyle(YAML::EmitterStyle::Flow);
        tf_data["position"] = position;

        // **修正 Orientation 存储**
        YAML::Node orientation;
        for (int r = 0; r < 3; ++r)
        {
            YAML::Node row;
            for (int c = 0; c < 3; ++c)
            {
                row.push_back(tf(r, c));
            }
            row.SetStyle(YAML::EmitterStyle::Flow);
            orientation.push_back(row);
        }
        tf_data["orientation"] = orientation;

        // **保存到 YAML 结构**
        root[name]["target_pose"] = tf_data;
    }

    // **写入 YAML 文件**
    std::ofstream file_out(filename);
    if (file_out.is_open())
    {
        file_out << root;
        file_out.close();
        ROS_INFO_STREAM("Saved computed transformations to " << filename);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open file " << filename);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_transform_from_yaml");
    ros::NodeHandle nh;

    YAML::Node floating_base_yaml = loadYAML(floating_base_file);
    YAML::Node tf_using_yaml = loadYAML(tf_using_file);

    if (!floating_base_yaml || !tf_using_yaml)
    {
        ROS_ERROR("Failed to load one or both YAML files.");
        return -1;
    }

    // **读取 init_floating_base（world -> base）**
    Eigen::Vector3f pos_world_base_init(floating_base_yaml["init_floating_base"][0].as<float>(), floating_base_yaml["init_floating_base"][1].as<float>(), floating_base_yaml["init_floating_base"][2].as<float>());
    Eigen::Matrix3f rot_world_base_init = quaternionToRotationMatrix(floating_base_yaml["init_floating_base"][3].as<float>(), floating_base_yaml["init_floating_base"][4].as<float>(), floating_base_yaml["init_floating_base"][5].as<float>(), floating_base_yaml["init_floating_base"][6].as<float>());
    Eigen::Matrix4f init_tf_mat_world_base = Eigen::Matrix4f::Identity();
    init_tf_mat_world_base.block<3, 3>(0, 0) = rot_world_base_init;
    init_tf_mat_world_base.block<3, 1>(0, 3) = pos_world_base_init;

    // **读取 gold_floating_base（world -> base）**
    Eigen::Vector3f pos_world_base_gold(floating_base_yaml["gold_floating_base"][0].as<float>(), floating_base_yaml["gold_floating_base"][1].as<float>(), floating_base_yaml["gold_floating_base"][2].as<float>());
    Eigen::Matrix3f rot_world_base_gold = quaternionToRotationMatrix(floating_base_yaml["gold_floating_base"][3].as<float>(), floating_base_yaml["gold_floating_base"][4].as<float>(), floating_base_yaml["gold_floating_base"][5].as<float>(), floating_base_yaml["gold_floating_base"][6].as<float>());
    Eigen::Matrix4f gold_tf_mat_world_base = Eigen::Matrix4f::Identity();
    gold_tf_mat_world_base.block<3, 3>(0, 0) = rot_world_base_gold;
    gold_tf_mat_world_base.block<3, 1>(0, 3) = pos_world_base_gold;

    std::vector<Eigen::Matrix4f> interpolated_poses;
    int point = 1000;

    // 提取平移
    Eigen::Vector3f pos_init = init_tf_mat_world_base.block<3, 1>(0, 3);
    Eigen::Vector3f pos_gold = gold_tf_mat_world_base.block<3, 1>(0, 3);

    // 提取旋转（转换为四元数）
    Eigen::Matrix3f rot_init = init_tf_mat_world_base.block<3, 3>(0, 0);
    Eigen::Matrix3f rot_gold = gold_tf_mat_world_base.block<3, 3>(0, 0);
    Eigen::Quaternionf q_init(rot_init);
    Eigen::Quaternionf q_gold(rot_gold);

    // 循环插值
    for (int i = 0; i < point; ++i)
    {
        float t = static_cast<float>(i) / (point - 1);  // t ∈ [0,1]

        // 平移插值
        Eigen::Vector3f pos_interp = (1 - t) * pos_init + t * pos_gold;

        // 旋转插值（SLERP）
        Eigen::Quaternionf q_interp = q_init.slerp(t, q_gold);
        Eigen::Matrix3f rot_interp = q_interp.toRotationMatrix();

        // 构造插值变换矩阵
        Eigen::Matrix4f tf_interp = Eigen::Matrix4f::Identity();
        tf_interp.block<3, 3>(0, 0) = rot_interp;
        tf_interp.block<3, 1>(0, 3) = pos_interp;

        interpolated_poses.push_back(tf_interp);
    }

    // **读取 TF 变换**
    Eigen::Matrix4f tf_mat_world_flan1 = parseTransformMatrix(tf_using_yaml["tf_mat_world_flan1"]);
    Eigen::Matrix4f tf_mat_world_flan4 = parseTransformMatrix(tf_using_yaml["tf_mat_world_flan4"]);
    Eigen::Matrix4f tf_mat_base_link1_0 = parseTransformMatrix(tf_using_yaml["tf_mat_base_link1_0"]);
    Eigen::Matrix4f tf_mat_base_link4_0 = parseTransformMatrix(tf_using_yaml["tf_mat_base_link4_0"]);

    // YAML 输出节点
    YAML::Node output_yaml;

    for (int i = 0; i < point; ++i)
    {
        const Eigen::Matrix4f& tf_interp = interpolated_poses[i];

        // 计算 base 坐标系下 flan1 和 flan4
        Eigen::Matrix4f tf_mat_base_flan1 = tf_interp.inverse() * tf_mat_world_flan1;
        Eigen::Matrix4f tf_mat_base_flan4 = tf_interp.inverse() * tf_mat_world_flan4;

        // 计算 link1_0 下的 flan1
        Eigen::Matrix4f tf_mat_link1_0_flan1 = tf_mat_base_link1_0.inverse() * tf_mat_base_flan1;

        // 计算 link4_0 下的 flan4
        Eigen::Matrix4f tf_mat_link4_0_flan4 = tf_mat_base_link4_0.inverse() * tf_mat_base_flan4;

        // 写入到 YAML 节点
        YAML::Node entry;
        for (int r = 0; r < 4; ++r)
        {
            YAML::Node row1, row4;
            for (int c = 0; c < 4; ++c)
            {
                row1.push_back(tf_mat_link1_0_flan1(r, c));
                row4.push_back(tf_mat_link4_0_flan4(r, c));
            }
            entry["tf_mat_link1_0_flan1"].push_back(row1);
            entry["tf_mat_link4_0_flan4"].push_back(row4);
        }

        // 添加时间步（或序号）作为键
        output_yaml["step_" + std::to_string(i)] = entry;
    }

    // 最终写入 YAML 文件
    std::ofstream fout(output_file);
    fout << output_yaml;
    fout.close();

    ROS_INFO("All interpolated transformations saved to leg_ik_cs.yaml!");
    ros::spin();
    return 0;
}
