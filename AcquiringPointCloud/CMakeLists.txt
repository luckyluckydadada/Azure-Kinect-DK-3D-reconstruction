# Enable C++11
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(AcquiringPointCloud)

# Kinect DK相机
find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
# eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")
# pcl
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )  #.h 文件
link_directories(${PCL_LIBRARY_DIRS}) # .so文件
#Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
#VTK
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
#OPENCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Declare the executable target built from your sources
add_executable(AcquiringPointCloud AcquiringPointCloud.cpp)

# Link your application with other libraries
target_link_libraries(AcquiringPointCloud k4a::k4a ${OpenCV_LIBS} libboost_system.so libboost_filesystem.so
${VTK_LIBRARIES} ${PCL_LIBRARIES})

