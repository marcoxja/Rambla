cmake_minimum_required(VERSION 3.8)
project(rambla_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smoke_topics.py TIMEOUT 60)
endif()

ament_package()
