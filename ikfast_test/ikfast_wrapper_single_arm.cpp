#define IKFAST_HAS_LIBRARY  // Build IKFast with API functions
#ifndef IKFAST_NO_MAIN
#define IKFAST_NO_MAIN  // Don't include main() from IKFast
#endif

/*
Set which IKFast version you are using
The API calls are slightly different for versions > 54
*/

#define IK_VERSION 61
#include "ikfast_single_arm.cpp"

// #define IK_VERSION 56
// #include "ikfast56.Transform6D.0_1_2_3_4_5.cpp"

// #define IK_VERSION 54
// #include "output_ikfast54.cpp"

//----------------------------------------------------------------------------//

#include <stdio.h>
#include <stdlib.h>
#include <time.h>  // for clock_gettime()
#include <vector>

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal  // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal  // for IKFast 54
#endif

namespace robots {
class Kinematics {
 public:
  int num_of_joints, num_free_parameters;
  Kinematics();
  ~Kinematics();
  std::vector<float> forward(std::vector<float> joint_config);
  std::vector<float> inverse(std::vector<float> ee_pose);
};
Kinematics::Kinematics() {
#if IK_VERSION > 54
  // for IKFast 56,61
  num_of_joints = GetNumJoints();
  num_free_parameters = GetNumFreeParameters();
#else
  // for IKFast 54
  num_of_joints = getNumJoints();
  num_free_parameters = getNumFreeParameters();
#endif
}
Kinematics::~Kinematics() {}
std::vector<float> Kinematics::forward(std::vector<float> joint_config) {
  IKREAL_TYPE eerot[9], eetrans[3];
  std::vector<float> ee_pose;

  if (joint_config.size() != num_of_joints) {
    printf(
        "\nError: (forward kinematics) expects vector of %d values describing "
        "joint angles (in radians).\n\n",
        num_of_joints);
    return ee_pose;
  }

  // Put input joint values into array
  IKREAL_TYPE joints[num_of_joints];
  for (unsigned int i = 0; i < num_of_joints; i++) {
    joints[i] = joint_config[i];
  }

#if IK_VERSION > 54
  // for IKFast 56,61
  ComputeFk(joints, eetrans, eerot);  // void return
#else
  // for IKFast 54
  fk(joints, eetrans, eerot);  // void return
#endif
  for (unsigned int i = 0; i < 3; i++) {
    ee_pose.push_back(eerot[i * 3 + 0]);
    ee_pose.push_back(eerot[i * 3 + 1]);
    ee_pose.push_back(eerot[i * 3 + 2]);
    ee_pose.push_back(eetrans[i]);
  }

  return ee_pose;
}
std::vector<float> Kinematics::inverse(std::vector<float> ee_pose) {
  IKREAL_TYPE eerot[9], eetrans[3];
  std::vector<float> joint_configs;

  if (ee_pose.size() == 7)  // ik, given translation vector and quaternion pose
  {
#if IK_VERSION > 54
    // for IKFast 56,61
    IkSolutionList<IKREAL_TYPE> solutions;
#else
    // for IKFast 54
    std::vector<IKSolution> vsolutions;
#endif
    std::vector<IKREAL_TYPE> vfree(num_free_parameters);

    eetrans[0] = ee_pose[0];
    eetrans[1] = ee_pose[1];
    eetrans[2] = ee_pose[2];

    // Convert input effector pose, in w x y z quaternion notation, to rotation
    // matrix. Must use doubles, else lose precision compared to directly
    // inputting the rotation matrix.
    double qw = ee_pose[3];
    double qx = ee_pose[4];
    double qy = ee_pose[5];
    double qz = ee_pose[6];
    const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    eerot[1] = 2.0f * qx * qy - 2.0f * qz * qw;
    eerot[2] = 2.0f * qx * qz + 2.0f * qy * qw;
    eerot[3] = 2.0f * qx * qy + 2.0f * qz * qw;
    eerot[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    eerot[5] = 2.0f * qy * qz - 2.0f * qx * qw;
    eerot[6] = 2.0f * qx * qz - 2.0f * qy * qw;
    eerot[7] = 2.0f * qy * qz + 2.0f * qx * qw;
    eerot[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;

    // For debugging, output the matrix
    /*
    for (unsigned char i=0; i<=8; i++)
    {   // detect -0.0 and replace with 0.0
        if ( ((int&)(eerot[i]) & 0xFFFFFFFF) == 0) eerot[i] = 0.0;
    }
    printf("     Rotation     %f   %f   %f  \n", eerot[0], eerot[1], eerot[2] );
    printf("                  %f   %f   %f  \n", eerot[3], eerot[4], eerot[5] );
    printf("                  %f   %f   %f  \n", eerot[6], eerot[7], eerot[8] );
    printf("\n");
    */

    // for(std::size_t i = 0; i < vfree.size(); ++i)
    //     vfree[i] = atof(argv[13+i]);

#if IK_VERSION > 54
    // for IKFast 56,61
    bool bSuccess = ComputeIk(eetrans, eerot,
                              vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
    // for IKFast 54
    bool bSuccess =
        ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif
    if (!bSuccess) {
      fprintf(stderr,
              "Error: (inverse kinematics) failed to get ik solution\n");
      return joint_configs;
    }

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
    // for IKFast 54
    unsigned int num_of_solutions = (int)vsolutions.size();
#endif
    // printf("Found %d ik solutions:\n", num_of_solutions );

    std::vector<IKREAL_TYPE> solvalues(num_of_joints);
    for (std::size_t i = 0; i < num_of_solutions; ++i) {
#if IK_VERSION > 54
      // for IKFast 56,61
      const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
      int this_sol_free_params = (int)sol.GetFree().size();
#else
      // for IKFast 54
      int this_sol_free_params = (int)vsolutions[i].GetFree().size();
#endif
      // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
      std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

#if IK_VERSION > 54
      // for IKFast 56,61
      sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
#else
      // for IKFast 54
      vsolutions[i].GetSolution(&solvalues[0],
                                vsolfree.size() > 0 ? &vsolfree[0] : NULL);
#endif

      for (std::size_t j = 0; j < solvalues.size(); ++j)
        joint_configs.push_back(solvalues[j]);
      // printf("%.15f, ", solvalues[j]);
      // printf("\n");
    }

  } else if (ee_pose.size() == 12)  // ik, given rotation-translation matrix
  {
#if IK_VERSION > 54
    // for IKFast 56,61
    IkSolutionList<IKREAL_TYPE> solutions;
#else
    // for IKFast 54
    std::vector<IKSolution> vsolutions;
#endif
    std::vector<IKREAL_TYPE> vfree(num_free_parameters);

    eerot[0] = ee_pose[0];
    eerot[1] = ee_pose[1];
    eerot[2] = ee_pose[2];
    eetrans[0] = ee_pose[3];
    eerot[3] = ee_pose[4];
    eerot[4] = ee_pose[5];
    eerot[5] = ee_pose[6];
    eetrans[1] = ee_pose[7];
    eerot[6] = ee_pose[8];
    eerot[7] = ee_pose[9];
    eerot[8] = ee_pose[10];
    eetrans[2] = ee_pose[11];
    // for(std::size_t i = 0; i < vfree.size(); ++i)
    //     vfree[i] = atof(argv[13+i]);

#if IK_VERSION > 54
    // for IKFast 56,61
    bool bSuccess = ComputeIk(eetrans, eerot,
                              vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
    // for IKFast 54
    bool bSuccess =
        ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif
    if (!bSuccess) {
      fprintf(stderr,
              "Error: (inverse kinematics) failed to get ik solution\n");
      return joint_configs;
    }

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
    // for IKFast 54
    unsigned int num_of_solutions = (int)vsolutions.size();
#endif
    // printf("Found %d ik solutions:\n", num_of_solutions );

    std::vector<IKREAL_TYPE> solvalues(num_of_joints);
    for (std::size_t i = 0; i < num_of_solutions; ++i) {
#if IK_VERSION > 54
      // for IKFast 56,61
      const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
      int this_sol_free_params = (int)sol.GetFree().size();
#else
      // for IKFast 54
      int this_sol_free_params = (int)vsolutions[i].GetFree().size();
#endif
      // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
      std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

#if IK_VERSION > 54
      // for IKFast 56,61
      sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
#else
      // for IKFast 54
      vsolutions[i].GetSolution(&solvalues[0],
                                vsolfree.size() > 0 ? &vsolfree[0] : NULL);
#endif
      for (std::size_t j = 0; j < solvalues.size(); ++j)
        joint_configs.push_back(solvalues[j]);
      //     printf("%.15f, ", solvalues[j]);
      // printf("\n");
    }

  } else {
    printf(
        "\nError: (inverse kinematics) please specify transformation of end "
        "effector with one of the following formats:\n"
        "    1) A vector of 7 values: a 3x1 translation (tX), and a 1x4 "
        "quaternion (w + i + j + k)\n"
        "    2) A (row-major) vector of 12 values: a 3x4 rigid transformation "
        "matrix with a 3x3 rotation R (rXX), and a 3x1 translation (tX)\n\n");
    return joint_configs;
  }

  return joint_configs;
}
}  // namespace robots

float SIGN(float x) { return (x >= 0.0f) ? +1.0f : -1.0f; }

float NORM(float a, float b, float c, float d) {
  return sqrt(a * a + b * b + c * c + d * d);
}
