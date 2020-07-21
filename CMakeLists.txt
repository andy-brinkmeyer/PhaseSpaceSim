cmake_minimum_required (VERSION 3.8)

project ("PSS"  VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# sub-projects
add_subdirectory ("PSS")

# build testing
option(BUILD_TESTS "Select to build unit tests for PSS_lib." ON)
if (BUILD_TESTS)
	add_subdirectory("testing")
endif()

# build estimation
option(BUILD_ESTIMATION "Select to build MoCap IMU estimation sensor fusion algorithms." ON)
if (BUILD_ESTIMATION)
	add_subdirectory("estimation")
endif()

