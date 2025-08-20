cmake_minimum_required(VERSION 3.8)
project(self_aiming)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV 4 REQUIRED)

include_directories(./include)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(self_aiming ${SOURCES})

ament_target_dependencies(self_aiming PUBLIC rclcpp cv_bridge sensor_msgs image_transport)

target_link_libraries(self_aiming PUBLIC ${OpenCV_LIBS})
target_include_directories(self_aiming PUBLIC ${OpenCV_INCLUDE_DIRS})

install(TARGETS self_aiming
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

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

ament_package()
