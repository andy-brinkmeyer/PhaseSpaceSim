cmake_minimum_required (VERSION 3.8)

# define the library
add_library(PSS_lib STATIC Camera.h Camera.cpp "test.cpp")
target_link_libraries(PSS_lib Boost::serialization ${OpenCV_LIBS} gtsam)

# define the test executable
add_executable(PSS_test test.cpp)
target_include_directories(PSS_test PUBLIC ${CMAKE_SOURCE_DIR}/PSS)
target_link_libraries(PSS_test PSS_lib Boost::serialization ${OpenCV_LIBS} gtsam)
