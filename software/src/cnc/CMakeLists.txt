cmake_minimum_required(VERSION 2.8.3)
project(cnc)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)
if (DEFINED "ENV{SOCEDS_DEST_ROOT}")
    message(STATUS "cross compiling for arm")
    # we are cross compiling for arm (make sure to be in embedded_command_shell.sh)
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR armv7l)

    set(compiler_path /opt/gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf/bin)
    set(CMAKE_C_COMPILER ${compiler_path}/arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER ${compiler_path}/arm-linux-gnueabihf-g++)
    #==============================================================================
    # Dependencies
    #==============================================================================
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            control_msgs
            sensor_msgs
            )
    include(findros)

    include_directories(/usr/include)

    find_package(PkgConfig REQUIRED)

    catkin_package(
            )

    SET(ALT_DEVICE_FAMILY soc_cv_av)
    add_definitions(-D${ALT_DEVICE_FAMILY})
    MESSAGE(STATUS "fpga include dir: $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/${ALT_DEVICE_FAMILY}")

    include_directories(
            include/cnc
            ${catkin_INCLUDE_DIRS}
            $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/${ALT_DEVICE_FAMILY}
            $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include
            $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include/${ALT_DEVICE_FAMILY}
    )
    #==============================================================================
    # Application
    #==============================================================================

    add_executable( cnc src/cnc.cpp )
    target_link_libraries( cnc -lpthread -lrt ${ros_LIBRARIES} )

    #==============================================================================
    # Installation
    #==============================================================================
    install(TARGETS
            cnc
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

    install(FILES
            package.xml
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
            )
else ()
    message(STATUS "cannot cross compile, are you in an embedded_command_shell.sh?!")
endif ()
