cmake_minimum_required (VERSION 3.8)

# find GTSAM
# you migth have to set GTSAM_DIR to the location of your GTSAM CMake directory
option(Boost_USE_STATIC_LIBS ON "Use static boost libraries. GTSAM uses static boost libraries on windows by default.")
find_package(GTSAM REQUIRED)

# define targets
add_executable(PSS_estimation
	"SimplePoseGraph.cpp")

target_include_directories(PSS_estimation PUBLIC
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/PSS/3rdparty
	${GTSAM_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}/gtsam/3rdparty
	Boost::headers
)

target_link_libraries(PSS_estimation
	PSS_lib
	gtsam
	gtsam_unstable
)

set_target_properties(PSS_estimation PROPERTIES LINKER_LANGUAGE CXX)