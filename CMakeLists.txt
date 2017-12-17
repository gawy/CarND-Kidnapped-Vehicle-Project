project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources_core src/particle_filter.cpp)
set(sources src/main.cpp ${sources_core})


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(particle_filter ${sources})


target_link_libraries(particle_filter z ssl uv uWS)

# adding test executable

enable_testing()
add_subdirectory(test)
include_directories(${gtest_SOURCE_DIR}/include)
include_directories(src)

file(GLOB SRC_TEST ${PROJECT_SOURCE_DIR}/test/*.cpp)
add_executable(gtest_suite ${SRC_TEST} ${sources_core})
target_link_libraries(gtest_suite gtest_main z ssl uv uWS)
add_test(NAME gtest-run COMMAND gtest_suite)

