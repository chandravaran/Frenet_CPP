cmake_minimum_required(VERSION 3.5)
project(mpc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  Eigen3
  OpenCV)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(
  ${EIGEN3_LIB_DIRS}
  /usr/local/lib
)

# Create Cpp executable
add_executable(frenet_test src/test_frenet.cpp )

target_link_libraries(frenet_test ${OpenCV_LIBS})


# ament_target_dependencies(frenet_test
#   rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros waypoint_msg OsqpEigen 
# )
# Install Cpp executables
install(TARGETS
  frenet_test
  DESTINATION lib/${PROJECT_NAME})
