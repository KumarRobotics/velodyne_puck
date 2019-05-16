cmake_minimum_required(VERSION 3.5)
project(velodyne_puck_driver)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        nodelet
                        diagnostic_updater
                        velodyne_puck_msgs)

catkin_package()

# Velodyne Puck driver
add_library(${PROJECT_NAME} src/driver.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Velodyne Puck nodelet
add_library(${PROJECT_NAME}_nodelet src/driver_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet PUBLIC ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
