cmake_minimum_required(VERSION 3.8)
project(aer_auto_exposure_gradient)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_CXX_FLAGS "-std=c++23 ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
)

# include_directories(
#  include
#   ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
# )

add_library(${PROJECT_NAME} src/auto_exp.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

add_executable(exp_node1 src/exp_node1.cpp)
target_link_libraries(exp_node1  ${PROJECT_NAME})
#add_dependencies(exp_node1 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
ament_target_dependencies(exp_node1 ${DEPENDENCIES})
install(TARGETS exp_node1 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_package()
