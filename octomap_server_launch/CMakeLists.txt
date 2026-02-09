cmake_minimum_required(VERSION 3.0.2)
project(octomap_server_launch)

find_package(catkin REQUIRED)
find_package(badger_salt_cmake)
find_package(badger_systemd)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_server_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install_salt_pillar_dir(salt_srv/pillar)
install_salt_state_dir(salt_srv/salt)

badger_install_systemd_service(FILES
  systemd/ros.octomap-server-map-frame.service
  systemd/ros.octomap-server-odom-frame.service
)
