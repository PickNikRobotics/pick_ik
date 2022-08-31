include(FetchContent)

set(CATCH_INSTALL_DOCS OFF)
set(CATCH_INSTALL_EXTRAS OFF)
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v${Catch2_FIND_VERSION})
FetchContent_MakeAvailable(Catch2)
set_target_properties(Catch2 PROPERTIES COMPILE_OPTIONS "")
include(Catch)
