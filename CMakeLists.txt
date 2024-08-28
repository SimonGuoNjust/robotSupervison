cmake_minimum_required(VERSION 3.10)  # 必须
set(CMAKE_CXX_STANDARD 14)	# C++14
set(CMAKE_BUILD_TYPE "Debug")
project(robotSupervision)  #工程名

cmake_policy(SET CMP0084 NEW)
set(CMAKE_PREFIX_PATH "D:/Qt/5.12.12/mingw73_64/bin")
list(APPEND CMAKE_PREFIX_PATH E:/opencv-4.8.1/build/install/x64/mingw/bin)


# message(STATUS, $ENV{Qt5})
set(CMAKE_AUTOMOC ON)   #QObject派生的C++类也需要通过moc进行处理 生成目录下有Demo_automoc文件(去掉才能编过)            
set(CMAKE_AUTOUIC ON)

set(VTK_DIR D:/vtk/prefix_x64/lib/cmake/vtk-8.2)
set(PCL_DIR D:/pcl/install/share/pcl-1.13)
# include_directories() # 头文件目录
find_package(Qt5 COMPONENTS Core Widgets Gui Qml Quick REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
# link_directories("/usr/local/lib/") # 链接库目录

# aux_source_directory(. SRCS) # 源文件
# aux_source_directory(./abc SRCS)

# set(CMAKE_C_FLAGS "-O1") # 设置C工程的 CFLAGS
set(CMAKE_CXX_FLAGS "-O0 -Werror") # 设置C++ 工程的 CXX_FLAGS
# message(STATUS, $ENV{BOOST_ROOT})
file(GLOB file_ui "./src/*.ui")
QT5_WRAP_UI(project_FORMS_HEADERS "mainwindow.ui")
INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    E:/opencv-4.8.1/build/install/include
    "D:/ffmpeg/include"
    "./src/")
file(GLOB FFMPEG_LIBS D:/ffmpeg/lib/*.lib)
add_executable(${PROJECT_NAME} ./src/main.cpp) # 生成可执行文件，这里程序名即为功能名
target_sources(${PROJECT_NAME} PRIVATE ${project_FORMS_HEADERS} 
./src/mainwindow.cpp 
./src/pullStream.cpp
./src/connect.cpp
./src/pcdecoder.cpp)
link_directories("")
target_link_libraries(${PROJECT_NAME}
${FFMPEG_LIBS}
${Boost_LIBRARIES}
${VTK_LIBRARIES}
${PCL_LIBRARIES}
E:/opencv-4.8.1/build/install/x64/mingw/bin/libopencv_core481.dll
E:/opencv-4.8.1/build/install/x64/mingw/bin/libopencv_highgui481.dll
E:/opencv-4.8.1/build/install/x64/mingw/bin/libopencv_imgproc481.dll
 "ws2_32.lib"
 Qt5::Core
 Qt5::Gui
 Qt5::Qml
 Qt5::Widgets
 Qt5::Quick) # 链接库

# 下面使用install作为项目打包使用
# set(CMAKE_INSTALL_PREFIX ./dist)  # 自定义安装目录，打包使用
# install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin) # 打包二进制文件

# set(CONFIGS ${PROJECT_SOURCE_DIR}/hello.cf) 
# install(FILES ${CONFIGS} DESTINATION config) # 打包配置文件
