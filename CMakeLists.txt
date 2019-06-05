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
                        message_generation
                        diagnostic_updater
                        sensor_msgs
                        std_msgs)
find_package(OpenMP)

add_message_files(DIRECTORY msg FILES VelodynePacket.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS
               src
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               pcl_ros
               cv_bridge
               image_transport
               sensor_msgs
               DEPENDS)

# Velodyne Puck Decoder
add_library(${PROJECT_NAME}
            src/driver.cpp
            src/driver_nodelet.cpp
            src/decoder.cpp
            src/decoder_nodelet.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
