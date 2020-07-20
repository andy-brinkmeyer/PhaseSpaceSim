cmake_minimum_required (VERSION 3.8)

project ("PSS"  VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# global dependencies
# find boost
# set(Boost_DEBUG ON)
# set(Boost_USE_STATIC_LIBS ON)
# set(Boost_ADDITIONAL_VERSIONS "1.73" "1.73.0")
# find_package(Boost 1.43 REQUIRED COMPONENTS serialization)

# find GTSAM
# you migth have to set GTSAM_DIR to the location of your GTSAM CMake directory
# find_package(GTSAM REQUIRED)
# set(Eigen_INCLUDE_DIR ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen)

# define PSS include directory
set(PSS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/PSS)

# sub-projects
add_subdirectory ("PSS")

# use testing
option(BUILD_TESTS "Select to build unit tests for PSS_lib." ON)
if (BUILD_TESTS)
	add_subdirectory("testing")
endif()

# executable for some dev testing
add_executable(TEST_EXE "test.cpp")
target_include_directories(TEST_EXE PUBLIC "PSS/3rdparty")
