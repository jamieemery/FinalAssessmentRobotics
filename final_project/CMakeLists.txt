cmake_minimum_required(VERSION 2.8.3)
project(final_project)

# Find catkin
find_package(catkin REQUIRED)

catkin_package()

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install move_base_config files
install(DIRECTORY move_base_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install world files
install(DIRECTORY world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install the rviz files
install(FILES multi_robot.rviz single_robot.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
