cmake_minimum_required(VERSION 3.8)
project(assignment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(trajectory src/trajectory.cpp src/assignment1.cpp)
ament_target_dependencies(trajectory rclcpp sensor_msgs geometry_msgs trajectory_msgs tf2 )

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  )
install( TARGETS trajectory DESTINATION lib/${PROJECT_NAME} )


ament_package()
