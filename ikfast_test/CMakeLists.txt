cmake_minimum_required(VERSION 3.10)
project(UR5IKSolver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加源文件
add_executable(ik_solver main.cpp)
add_executable(ik_ec_66_wrapper main_ec66_wrapper.cpp)
add_executable(ik_ec_66 main_ec66.cpp)
add_executable(ik_arm_wrapper main_arm_wrapper.cpp)
add_executable(ik_arm main_arm.cpp)
add_executable(ik_single_arm main_single_arm.cpp)
add_executable(ik_single_arm_wrapper main_single_arm_wrapper.cpp)
add_executable(ik_single_arm_test main_single_arm_test.cpp)

# 禁用 ikfast61.cpp 中的 main 函数
target_compile_definitions(ik_solver PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_ec_66_wrapper PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_ec_66 PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_arm_wrapper PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_arm PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_single_arm PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_single_arm_wrapper PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(ik_single_arm_test PRIVATE IKFAST_NO_MAIN)

# 包含头文件目录
target_include_directories(ik_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_ec_66_wrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_ec_66 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_arm_wrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_single_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_single_arm_wrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ik_single_arm_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 链接数学库
target_link_libraries(ik_solver m)
target_link_libraries(ik_ec_66_wrapper m)
target_link_libraries(ik_ec_66 m)
target_link_libraries(ik_arm_wrapper m)
target_link_libraries(ik_arm m)
target_link_libraries(ik_single_arm m)
target_link_libraries(ik_single_arm_wrapper m)
target_link_libraries(ik_single_arm_test m)

# 查找并链接 LAPACK 和 BLAS 库
find_package(LAPACK REQUIRED)
if(LAPACK_FOUND)
    target_link_libraries(ik_solver ${LAPACK_LIBRARIES})
    target_link_libraries(ik_ec_66_wrapper ${LAPACK_LIBRARIES})
    target_link_libraries(ik_ec_66 ${LAPACK_LIBRARIES})
    target_link_libraries(ik_arm_wrapper ${LAPACK_LIBRARIES})
    target_link_libraries(ik_arm ${LAPACK_LIBRARIES})
    target_link_libraries(ik_single_arm ${LAPACK_LIBRARIES})
    target_link_libraries(ik_single_arm_wrapper ${LAPACK_LIBRARIES})
    target_link_libraries(ik_single_arm_test ${LAPACK_LIBRARIES})
else()
    message(FATAL_ERROR "未找到 LAPACK 库")
endif()

find_package(BLAS REQUIRED)
if(BLAS_FOUND)
    target_link_libraries(ik_solver ${BLAS_LIBRARIES})
    target_link_libraries(ik_ec_66_wrapper ${BLAS_LIBRARIES})
    target_link_libraries(ik_ec_66 ${BLAS_LIBRARIES})
    target_link_libraries(ik_arm_wrapper ${BLAS_LIBRARIES})
    target_link_libraries(ik_arm ${BLAS_LIBRARIES})
    target_link_libraries(ik_single_arm ${BLAS_LIBRARIES})
    target_link_libraries(ik_single_arm_wrapper ${BLAS_LIBRARIES})
    target_link_libraries(ik_single_arm_test ${BLAS_LIBRARIES})
else()
    message(FATAL_ERROR "未找到 BLAS 库")
endif()


