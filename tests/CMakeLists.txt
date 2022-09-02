list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Catch2 3.1.0 REQUIRED)

add_executable(goal_tests goal_tests.cpp)
target_link_libraries(goal_tests PRIVATE gd_ik_plugin Catch2::Catch2WithMain)
catch_discover_tests(goal_tests)

add_executable(math_tests math_tests.cpp)
target_link_libraries(math_tests PRIVATE gd_ik_plugin Catch2::Catch2WithMain)
catch_discover_tests(math_tests)
