cmake_minimum_required(VERSION 3.5)
project(velodyne_puck_decoder)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        nodelet
                        pluginlib
                        sensor_msgs
                        pcl_ros
                        pcl_conversions
                        velodyne_puck_msgs)

catkin_package()

# Velodyne Puck Decoder
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME}
                           PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Velodyne Puck Decoder nodelet
add_library(${PROJECT_NAME}_nodelet src/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet PUBLIC ${PROJECT_NAME})
add_dependencies(velodyne_puck_decoder_nodelet
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
