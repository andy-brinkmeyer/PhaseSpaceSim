cmake_minimum_required (VERSION 3.8)

# set some compiler flags
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# create targets
add_library(PSS_lib STATIC
	"camera/LinearDetector.h" "camera/LinearDetector.cpp"
	"camera/Camera.h" "camera/Camera.cpp"
	"core/Core.h" "core/Core.cpp"
	"core/SimulationContext.h" "core/SimulationContext.cpp"
	"geometry/Pose3.h" "geometry/Pose3.cpp" 
	"geometry/Rot3.h" "geometry/Rot3.cpp")

set_target_properties(PSS_lib PROPERTIES 
	LINKER_LANGUAGE CXX
	)
target_include_directories(PSS_lib PUBLIC "${PSS_INCLUDE_DIR}/3rdparty")

