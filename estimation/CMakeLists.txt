cmake_minimum_required (VERSION 3.8)

# find GTSAM
# you migth have to set GTSAM_DIR to the location of your GTSAM CMake directory
option(Boost_USE_STATIC_LIBS ON "Use static boost libraries. GTSAM uses static boost libraries on windows by default.")
find_package(GTSAM REQUIRED)

# for MSVC increase /STACK size
# GTSAM uses some operations that might lead to stack overflows when our graph grows too large
if(MSVC)
	set(STACK_SIZE "/STACK:4194304")
endif()

# define targets
add_library(PSS_factors SHARED
	"PSS/factors/LinearDetectorFactor.h" "PSS/factors/LinearDetectorFactor.cpp")

target_link_libraries(PSS_factors
	PUBLIC PSS_lib
	PUBLIC gtsam
	PUBLIC gtsam_unstable
)

target_include_directories(PSS_factors PUBLIC
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/PSS/3rdparty
	${GTSAM_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}/gtsam/3rdparty
	Boost::headers)

add_executable(PSS_estimation
	"SimplePoseGraph.cpp")

target_include_directories(PSS_estimation PRIVATE
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/estimation
	${CMAKE_SOURCE_DIR}/PSS/3rdparty
	${GTSAM_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}/gtsam/3rdparty
	Boost::headers)

target_link_libraries(PSS_estimation
	PRIVATE PSS_lib
	PRIVATE PSS_factors
	PRIVATE gtsam
	PRIVATE gtsam_unstable
)

set_target_properties(PSS_estimation PROPERTIES 
	LINKER_LANGUAGE CXX
	LINK_FLAGS ${STACK_SIZE}
)