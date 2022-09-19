cmake_minimum_required(VERSION 3.14)

add_executable(run_kitti_example run_kitti_example.cpp)
add_executable(plot_traj traj_plot.cpp)
target_link_libraries(run_kitti_example myslam ${THIRD_PARTY_LIBS})
target_link_libraries(plot_traj ${THIRD_PARTY_LIBS})