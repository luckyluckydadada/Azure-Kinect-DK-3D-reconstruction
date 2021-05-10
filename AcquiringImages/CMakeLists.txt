# Enable C++11
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(AcquiringImages)

# Kinect DK相机
find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
# Find OpenCV
find_package(OpenCV REQUIRED)

# Add includes
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Declare the executable target built from your sources
add_executable(AcquiringImages AcquiringImages.cpp)

# Link your application with other libraries
target_link_libraries(AcquiringImages k4a::k4a ${OpenCV_LIBS} )
