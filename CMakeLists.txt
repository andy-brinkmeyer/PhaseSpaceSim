cmake_minimum_required (VERSION 3.8)

project ("PSS"  VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# global dependencies
# find boost
set(Boost_DEBUG ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_ADDITIONAL_VERSIONS "1.73" "1.73.0")
find_package(Boost 1.43 REQUIRED COMPONENTS serialization)
include_directories(${Boost_INCLUDE_DIR})

# find opencv
# you might have to set OpenCV_DIR to the proper directory
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find GTSAM
# you migth have to set GTSAM_DIR to the location of your GTSAM CMake directory
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# sub-projects
add_subdirectory ("PSS")
