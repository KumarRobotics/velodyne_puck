cmake_minimum_required(VERSION 3.5)
project(velodyne_puck)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        nodelet
                        pluginlib
                        pcl_ros
                        pcl_conversions
                        cv_bridge
                        image_transport
                        diagnostic_updater
                        sensor_msgs
                        velodyne_msgs)

catkin_package(INCLUDE_DIRS src LIBRARIES ${PROJECT_NAME})

# Velodyne Puck Decoder
add_library(${PROJECT_NAME}
            src/driver.cpp
            src/driver_nodelet.cpp
            src/decoder.cpp
            src/decoder_nodelet.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
