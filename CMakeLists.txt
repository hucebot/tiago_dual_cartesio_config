cmake_minimum_required(VERSION 3.5)
project(tiago_dual_cartesio_config)

find_package(ament_cmake REQUIRED)


# Install launch files.
install(DIRECTORY
  capsules launch python robots stack
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

