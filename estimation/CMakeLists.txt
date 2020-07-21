cmake_minimum_required (VERSION 3.8)

# dependencies
# find boost
set(Boost_DEBUG ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_ADDITIONAL_VERSIONS "1.73.0")
find_package(Boost 1.43 REQUIRED)

# find GTSAM
# you migth have to set GTSAM_DIR to the location of your GTSAM CMake directory
find_package(GTSAM REQUIRED)

# define targets
add_executable(PSS_estimation
	"SimplePoseGraph.cpp")
target_include_directories(PSS_estimation PUBLIC
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/PSS/3rdparty
	${GTSAM_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}/gtsam/3rdparty
)
target_link_libraries(PSS_estimation
	PSS_lib
	gtsam
)

set_target_properties(PSS_estimation PROPERTIES LINKER_LANGUAGE CXX)