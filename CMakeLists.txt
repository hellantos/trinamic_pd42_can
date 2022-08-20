cmake_minimum_required(VERSION 3.8)
project(trinamic_pd42_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(canopen REQUIRED)
find_package(lely_core_libraries REQUIRED)

generate_dcf(single-pd42)

# install launch file
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
