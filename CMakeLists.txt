cmake_minimum_required(VERSION 3.0)
project(Pcl_Demo)

set(CMAKE_BUILD_TYPE Debug)
#c++编译标准
set(CMAKE_CXX_STANDARD 11)
#生成的动态库存储位置
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


# set(RESOURCE_DIR resources/res.qrc)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIRD_INCLUDE_DIRS
		${OpenCV_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${Eigen3_INCLUDE_DIRS}
		)
set(THIRD_PARTY_LIBS
		pthread
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
		)

set(INTERNAL_INCLUDE_DIRS
		${PROJECT_SOURCE_DIR}/src
		)

set(INTERNAL_LIBS

		)

# 链接生成的动态库目录路径，类似于搜索路径
link_directories(${PROJECT_BINARY_DIR}/lib)
link_directories(${INTERNAL_LIBS})
#头文件目录
include_directories(${THIRD_INCLUDE_DIRS} ${INTERNAL_INCLUDE_DIRS})

enable_testing()

add_subdirectory(test)
add_subdirectory(src)

##############install##################
#安装动态库和静态库


#安装头文件

