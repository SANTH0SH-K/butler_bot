cmake_minimum_required(VERSION 3.8)
project(butler_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY  launch models urdf worlds config meshes test params maps scripts  
    DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/task_1.py
  scripts/task_2.py
  scripts/task_3.py
  scripts/task_4.py
  scripts/task_5.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
