list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Catch2 3.1.0 REQUIRED)

add_executable(test-algorithm algorithm_tests.cpp)
target_link_libraries(test-algorithm PRIVATE gd_ik_plugin Catch2::Catch2WithMain)
catch_discover_tests(test-algorithm)
