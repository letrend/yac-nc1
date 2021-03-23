cmake_minimum_required(VERSION 2.8.3)
project(cnc_gui)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs sensor_msgs)

find_package(VISP REQUIRED)

find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

include_directories(include qcustomplot)

## System dependencies are found with CMake's conventions

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
else ()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif ()

catkin_python_setup()

roslint_cpp()

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs sensor_msgs
        DEPENDS YAML_CPP)

include_directories(
        include/cnc_gui/
        ${CMAKE_CURRENT_BINARY_DIR}/..
        ${catkin_INCLUDE_DIRS}
        ${VISP_INCLUDE_DIRS}
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    qt5_wrap_cpp(cnc_gui_MOCS
            include/cnc_gui/cnc_gui.hpp
            qcustomplot/qcustomplot.h)
    qt5_wrap_ui(cnc_gui_UIS_H
            src/cnc_gui.ui
            )
else ()
    qt4_wrap_cpp(cnc_gui_MOCS
            cnc_gui.hpp
            qcustomplot/qcustomplot.h)
    qt4_wrap_ui(cnc_gui_UIS_H
            src/cnc_gui.ui
            )
endif ()



add_library(cnc_gui src/cnc_gui.cpp include/cnc_gui/pose_helper.h src/pose_helper.cpp
        src/brain_dice_config.cpp src/camera.cpp src/joystick.cpp
        ${cnc_gui_MOCS} ${cnc_gui_UIS_H}
        qcustomplot/qcustomplot.cpp )
target_link_libraries(cnc_gui ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} -ldl)

add_executable(test_visp_tracker src/test_visp_tracker.cpp include/cnc_gui/pose_helper.h src/pose_helper.cpp)
target_link_libraries(test_visp_tracker ${VISP_LIBRARIES} ${catkin_LIBRARIES} -ldl)

add_executable(camera_calibration src/camera_calibration.cpp)
target_link_libraries(camera_calibration  ${VISP_LIBRARIES}  -ldl)

add_executable(camera_viewer src/camera_viewer.cpp)
target_link_libraries(camera_viewer  ${VISP_LIBRARIES}  -ldl)

add_executable(test_brain_dice_config src/test_braindiceconfig.cpp src/brain_dice_config.cpp)
target_link_libraries(test_brain_dice_config  ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES}  -ldl)

add_executable(test_joystick src/test_joystick.cpp)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    target_link_libraries(cnc_gui Qt5::Widgets)
else ()
    target_link_libraries(cnc_gui ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif ()

#############
## Install ##
#############

install(TARGETS cnc_gui
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
