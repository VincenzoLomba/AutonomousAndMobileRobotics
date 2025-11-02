
cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)   # -> this MUST also be indicated in the package.xml file!
find_package(std_msgs REQUIRED) # the basic idea is that in the package.xml we have a full list of dependencies for the ROS package, meanwhile in here we tell the compiler how to do its job (and what libraries to use)

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

# Let's setup the 'talker' executable (it will be used to generate the talker Node)
add_executable(talker src/publisher_member_function.cpp) # create an executable (from which we'll be able to create nodes)
ament_target_dependencies(talker rclcpp std_msgs) #  indicate for the 'talker' executable what (pre-delcared) libraries it depends on
# Let's setup the 'subscriber' executable (it will be used to generate the subscriber Node)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

ament_package()

# lets make the 'talker' executable seenable by "ros2 run 'package_name'" command:
install(TARGETS
talker
DESTINATION lib/${PROJECT_NAME})
# Same for 'subscriber':
install(TARGETS
talker
listener
DESTINATION lib/${PROJECT_NAME})
