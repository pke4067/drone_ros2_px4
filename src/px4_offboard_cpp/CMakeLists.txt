cmake_minimum_required(VERSION 3.8)
project(px4_offboard_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 1. 'offboard_control.cpp' 파일을 'offboard_control'이라는 이름의 실행 프로그램으로 만듭니다.
add_executable(offboard_control src/offboard_control.cpp)

# 2. 이 프로그램이 작동하는 데 필요한 'ROS 2 도구(rclcpp)'와 '드론 단어장(px4_msgs)'을 연결합니다.
ament_target_dependencies(offboard_control
  rclcpp
  px4_msgs
)

# 3. 조립이 끝난 프로그램을 시스템에 설치해서 'ros2 run' 명령어로 쓸 수 있게 합니다.
install(TARGETS
  offboard_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
