cmake_minimum_required(VERSION 3.0.2)
project(zy_ka)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        livox_ros_driver
        )

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using Ceres version ${CERES_VERSION}")

catkin_package(
        # LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs
        DEPENDS EIGEN3 PCL OpenCV
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(
        include
        ${GTSAM_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(FormatConvert_zy src/FormatConvert.cpp)
target_link_libraries(FormatConvert_zy ${catkin_LIBRARIES})

add_executable(CustomMsgViewer src/CustomMsgViewer.cpp)
target_link_libraries(CustomMsgViewer ${catkin_LIBRARIES})

add_executable(Point_ana src/Point_ana.cpp src/pcl_viewer.cpp src/Entity_Cla.cpp src/FastGroundFit.cpp)
target_link_libraries(Point_ana ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(Point_ana1 src/Point_ana1.cpp src/pcl_viewer.cpp src/Entity_Cla.cpp src/FastGroundFit.cpp)
target_link_libraries(Point_ana1 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(zy_scanregistration src/zy_scanRegistration.cpp src/Entity_Cla.cpp src/FastGroundFit.cpp src/pcl_viewer.cpp)
target_link_libraries(zy_scanregistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(zy_livox_repub src/zy_livox_repub.cpp)
target_link_libraries(zy_livox_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(zy_normEstimate src/zy_normEstimate.cpp src/Entity_Cla.cpp src/FastGroundFit.cpp src/pcl_viewer.cpp)
target_link_libraries(zy_normEstimate ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(zy_LidarOdometry src/zy_LidarOdometry.cpp src/Entity_Cla.cpp src/FastGroundFit.cpp src/pcl_viewer.cpp)
target_link_libraries(zy_LidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}  ${OpenCV_LIBS})

#For Debug
set(CMAKE_BUILD_TYPE "Debug")
