cmake_minimum_required (VERSION 3.8)

# add the test target
add_executable(LinDetectorTest "LinearDetectorTest.cpp")
target_include_directories(LinDetectorTest PUBLIC
	${PSS_INCLUDE_DIR}
	${GTEST_INCLUDE_DIR}
)
target_link_libraries(LinDetectorTest
	GTest::GTest 
	GTest::Main
	gtsam
	Boost::serialization
	PSS_lib
)

# find the tests
add_test(LinTest LinDetectorTest)