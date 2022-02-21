cmake_minimum_required(VERSION 3.8)
project(spraybot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
        scripts/vectornav_html_status.py
        scripts/gps_datum.py
        scripts/pause_bagfiles.py
        scripts/waypoint_teleop.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable copyright and license checks
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable cpplint and manually call it with filters later
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
