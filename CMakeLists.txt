cmake_minimum_required(VERSION 3.8)
project(waypoint_reconfigure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# auto find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(waypoint_reconfigure
  src/waypoint_reconfigure_node.cpp
)

target_link_libraries(waypoint_reconfigure  yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_cmake_auto
)

target_link_libraries(${PROJECT_NAME}
  # 他の依存ライブラリ
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS waypoint_reconfigure
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)