cmake_minimum_required(VERSION 3.10)
project(RTSP_SERVER)
add_definitions("-Wall -g")
# 设置使用C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# libavdevice 
#link_directories(/home/danbao/ffmpeg-6.1.2/build/lib)
include_directories(./live555)
#  ./ffmpeg/include)

file(GLOB_RECURSE SOURCES "src/*.cpp")
# 添加可执行文件


file(GLOB_RECURSE SRC_DIR_LIST "src/*.cpp")

add_library(
        ${PROJECT_NAME} STATIC
        ${SRC_DIR_LIST}										#将DIR_LIB_SRCS中文件列表以静态库的方式加入工程
)


add_executable(rtsp_server  src/main.cpp)
target_link_libraries(rtsp_server ${PROJECT_NAME} )
target_link_libraries(rtsp_server pthread liveMedia BasicUsageEnvironment groupsock UsageEnvironment x264)
# avcodec avformat swscale avutil swresample)

