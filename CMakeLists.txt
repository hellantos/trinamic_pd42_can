cmake_minimum_required(VERSION 3.8)
project(trinamic_pd42_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(canopen_interfaces REQUIRED)

generate_dcf(single-pd42)

add_executable(position_tick_client src/position_tick_motor.cpp)
ament_target_dependencies(position_tick_client
  rclcpp std_srvs canopen_interfaces)


install(TARGETS
position_tick_client
DESTINATION lib/${PROJECT_NAME})

# install launch file
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
