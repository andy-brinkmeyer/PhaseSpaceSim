cmake_minimum_required (VERSION 3.8)

# add the test target
add_executable(PSS_test 
	"camera/LinearDetectorTest.cpp" 
	"camera/CameraTest.cpp"
	"core/CoreTest.cpp"
	"core/SimulationContextTest.cpp")
target_include_directories(PSS_test PUBLIC
	${PSS_INCLUDE_DIR}
	${GTEST_INCLUDE_DIR}
)
target_link_libraries(PSS_test
	GTest::GTest 
	GTest::Main
	gtsam
	Boost::serialization
	PSS_lib
)

set_target_properties(PSS_test PROPERTIES 
	LINKER_LANGUAGE CXX
	)

# find the tests
add_test(LinTest PSS_test)