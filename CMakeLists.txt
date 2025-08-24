cmake_minimum_required(VERSION 3.10)
project(qt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(QT_VERSION 5)
set(REQUIRED_LIBS Core Widgets DataVisualization)
set(REQUIRED_LIBS_QUALIFIED Qt5::Core Qt5::Widgets  Qt5::DataVisualization)
#FILE(GLOB UI_FILES "./*.ui")
#INCLUDE_DIRECTORIES(home/sophda/src/qtdatavis3d-opensource-src-5.9.5/examples/datavisualization)
# 通过Ui文件生成对应的头文件
#QT5_WRAP_UI(WRAP_FILES ${UI_FILES})
#add_executable(${PROJECT_NAME} main.cpp mainwindow.cpp mainwindow.h mainwindow.ui pcdProcess.cpp pcdProcess.h)
add_executable(${PROJECT_NAME} main.cpp mainwindow.cpp mainwindow.h mainwindow.ui threedim.ui
        src/pointCloud.cpp src/pointCloud.h src/MyView.cpp src/MyView.h
        src/circuitbasescene.h src/circuitbasescene.cpp src/MyItem.cpp
        src/MyItem.h src/Port.cpp src/Port.h src/Link.cpp src/Link.h src/NodeData.cpp
        src/NodeData.h src/Macro.h src/FuntionCloud.cpp src/FuntionCloud.h
        src/FunctionImage.cpp src/FunctionImage.h src/DBSCAN.cpp src/DBSCAN.h src/DimWindow.cpp src/DimWindow.h
        src/threedim.h)

#if (NOT CMAKE_PREFIX_PATH)
#    message(WARNING "CMAKE_PREFIX_PATH is not defined, you may need to set it "
#            "(-DCMAKE_PREFIX_PATH=\"path/to/Qt/lib/cmake\" or -DCMAKE_PREFIX_PATH=/usr/include/{host}/qt{version}/ on Ubuntu)")
#endif ()
find_package(VTK REQUIRED)
find_package(Qt${QT_VERSION} COMPONENTS ${REQUIRED_LIBS} REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
message(${PCL_LIBRARIES})

include(${VTK_USE_FILE})

#修改
#target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${VTK_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${REQUIRED_LIBS_QUALIFIED} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
