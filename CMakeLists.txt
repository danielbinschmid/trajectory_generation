cmake_minimum_required(VERSION 3.5)

project(crazyflie_trajectory)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crazyflie_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
    3rd/eigen# ${EIGEN3_INCLUDE_DIRS}
    include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# /usr/local/include
add_executable(traj_gen src/traj_gen.cpp)
ament_target_dependencies(traj_gen rclcpp crazyflie_msgs geometry_msgs sensor_msgs)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

install(
  TARGETS
  traj_gen
  DESTINATION lib/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME})



ament_package()
