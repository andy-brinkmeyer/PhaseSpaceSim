cmake_minimum_required (VERSION 3.8)

# create targets
add_library(PSS_lib STATIC
	"camera/LinearDetector.h" "camera/LinearDetector.cpp"
	 "camera/Camera.h" "camera/Camera.cpp"
	  "Core/Core.h" "Core/Core.cpp"
	  )
set_target_properties(PSS_lib PROPERTIES 
	LINKER_LANGUAGE CXX
	CXX_STANDARD 14
	)
target_include_directories(PSS_lib PUBLIC ${GTSAM_INCLUDE_DIR} ${Eigen_INCLUDE_DIR} ${Boost_INCLUDE_DIR} "3rdparty")

