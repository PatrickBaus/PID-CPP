cmake_minimum_required(VERSION 3.14)
project(PidTests CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# SANITIZE=ON builds the same tests with UndefinedBehaviorSanitizer and AddressSanitizer instead of the coverage
# instrumentation. Any finding aborts the test run.
option(SANITIZE "Build with -fsanitize=undefined,address instead of coverage instrumentation" OFF)

include(FetchContent)
# doctest is a single header. SOURCE_SUBDIR points to a non-existent directory
# so that only the sources are fetched and doctest's own build is not configured.
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG        v2.4.11
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  do-not-configure
)
FetchContent_MakeAvailable(doctest)

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)

if(SANITIZE)
  set(INSTRUMENTATION -fsanitize=undefined,address -fno-sanitize-recover=all -fno-omit-frame-pointer -g -O1)
else()
  set(INSTRUMENTATION --coverage -O0)
endif()

# The library under test. It has no dependency on the Arduino API, so no fakes are required.
add_library(pid STATIC ${LIBRARY_DIR}/pid.cpp)
target_include_directories(pid PUBLIC ${LIBRARY_DIR})
target_compile_options(pid PRIVATE -Wall -Wextra -Werror ${INSTRUMENTATION})

# The tests are instrumented as well, because the saturating maths helpers in pid.h are inline functions and the
# helpers which are not used by pid.cpp only exist in the test's translation unit.
add_executable(pid_tests test_pid.cpp)
target_include_directories(pid_tests SYSTEM PRIVATE ${doctest_SOURCE_DIR}/doctest)
target_compile_options(pid_tests PRIVATE -Wall -Wextra ${INSTRUMENTATION})
target_link_libraries(pid_tests PRIVATE pid ${INSTRUMENTATION})

enable_testing()
add_test(NAME pid_tests COMMAND pid_tests)
