cmake_minimum_required(VERSION 2.8.3)
project(horned_sungem_stream)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(LIBHS ../../SungemSDK)
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(LIBHS_LIB ${LIBHS}/lib/linux/${CMAKE_SYSTEM_PROCESSOR})
else()
    message(FATAL_ERROR "Unsupported platforms ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_package(catkin REQUIRED COMPONENTS
        nodelet
        roscpp
        cv_bridge
        image_transport
        object_msgs
        roslint
        rospy
        rostest
        sensor_msgs
        horned_sungem_lib
        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS cv_bridge horned_sungem_lib image_transport nodelet object_msgs roscpp roslint rospy rostest sensor_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${LIBHS}/include
)
link_directories(${LIBHS_LIB})

add_library(${PROJECT_NAME}
        src/hs_nodelet.cpp
        )



add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


target_link_libraries(${PROJECT_NAME}
        -lhs
        ${catkin_LIBRARIES}

        )


install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )