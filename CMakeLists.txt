cmake_minimum_required(VERSION 3.5)
project(velodyne_puck)

set(CMAKE_CXX_STANDARD 11)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             pcl_ros
             pcl_conversions
             cv_bridge
             image_transport
             diagnostic_updater
             dynamic_reconfigure
             sensor_msgs
             velodyne_msgs)

generate_dynamic_reconfigure_options(cfg/VelodynePuck.cfg)

catkin_package()

add_executable(${PROJECT_NAME}_driver src/driver.cpp)
target_include_directories(${PROJECT_NAME}_driver PUBLIC ${catkin_INCLUDE_DIRS}
                                                         src)
target_link_libraries(${PROJECT_NAME}_driver PUBLIC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_decoder src/decoder.cpp)
target_include_directories(${PROJECT_NAME}_decoder PUBLIC ${catkin_INCLUDE_DIRS}
                                                          src)
target_link_libraries(${PROJECT_NAME}_decoder PUBLIC ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_driver ${PROJECT_NAME}_gencfg)
