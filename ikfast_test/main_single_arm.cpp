#include <iostream>
#include "ikfast.h"  // Include the IKFast solver
#include "ikfast_single_arm.cpp"

using namespace ikfast;  // Use the ikfast namespace

int main() {
  // 定义原始的关节角度（弧度制）
  IkReal joints[6] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // 用于存储正向运动学计算得到的末端执行器位置和姿态
  IkReal eetrans[3];
  IkReal eerot[9];

  // 计算正向运动学
  ComputeFk(joints, eetrans, eerot);

  std::cout << "正向运动学计算结果：" << std::endl;
  std::cout << "位置：[" << eetrans[0] << ", " << eetrans[1] << ", "
            << eetrans[2] << "]" << std::endl;
  std::cout << "姿态矩阵：" << std::endl;
  for (int i = 0; i < 9; i++) {
    std::cout << eerot[i] << " ";
    if ((i + 1) % 3 == 0) {
      std::cout << std::endl;
    }
  }

  // 创建一个解列表来存储逆运动学解
  IkSolutionList<IkReal> solutions;

  // 计算逆运动学
  bool success = ComputeIk(eetrans, eerot, nullptr, solutions);

  if (!success) {
    std::cerr << "无法计算逆解。" << std::endl;
    return -1;
  }

  size_t num_of_solutions = solutions.GetNumSolutions();
  std::cout << "找到 " << num_of_solutions << " 个逆运动学解：" << std::endl;

  std::vector<IkReal> solvalues(GetNumJoints());
  for (size_t i = 0; i < num_of_solutions; ++i) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    sol.GetSolution(&solvalues[0], nullptr);

    std::cout << "解 " << i << ": ";
    for (size_t j = 0; j < solvalues.size(); ++j) {
      std::cout << solvalues[j] << " ";
    }
    std::cout << std::endl;
  }

  // 您可以在此处添加代码，对比原始关节角度和逆运动学解

  return 0;
}
