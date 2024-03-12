cmake_minimum_required(VERSION 3.5)

project(Display VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(catkin_DIR "/opt/ros/noetic/share/catkin/cmake")
set(VTK_DIR "/usr/local/include/vtk-9.3")
set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.13")
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rviz
  #pcl_conversions
  #pcl_ros
  sensor_msgs
  message_generation
  visualization_msgs
)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS OpenGLWidgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
#find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS OpenGLWidgets REQUIRED)
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS OpenGLWidgets REQUIRED)
else()
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS OpenGL REQUIRED)
endif()
#include VTK
find_package(VTK 9.3.0 REQUIRED COMPONENTS
    CommonCore
    GUISupportQt
)
#file(GLOB PCL_LIBRARIES /usr/local/lib/libpcl_*)
#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.11)

find_package(PCL REQUIRED COMPONENTS
    common
    visualization
    io
)
message("PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}, PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}, PCL_LIBRARIES=${PCL_LIBRARIES}")
if (NOT VTK_FOUND)
  message("Skipping example: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif ()

include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(${catkin_DIR})
include_directories(./)
#include_directories(/usr/local/include/vtk-9.3)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
#INCLUDE_DIRECTORIES(${PROJECT_NAME} ${PCL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        myscrolltextbar.cpp
        myscrolltextbar.h
        BoundingBox.h
        BoundingBox.cpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Display
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        myscrolltextbar.cpp
        myscrolltextbar.h

    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET Display APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(Display SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(Display
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(Display
    PRIVATE
    ${PCL_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${VTK_LIBRARIES}
    ${catkin_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
#target_link_libraries(Display PRIVATE Qt${QT_VERSION_MAJOR}::OpenGLWidgets)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.Display)
endif()
set_target_properties(Display PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS Display
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(Display)
endif()
