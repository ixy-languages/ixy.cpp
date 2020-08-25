cmake_minimum_required(VERSION 3.12)
project(ixy
        VERSION 0.1
        DESCRIPTION "C++ rewrite of the ixy network driver"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(
        -Wall
        -Wextra
        -pedantic
        -Wno-unused-parameter
        -Wno-unused-function
        -Wformat=2
)

add_subdirectory(deps/fmt)

add_library(ixy src/ixy.cpp src/ixgbe.cpp src/mempool.cpp src/memory.cpp src/pci.cpp)
target_include_directories(ixy PUBLIC ${PROJECT_SOURCE_DIR}/include PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(ixy PUBLIC fmt PRIVATE stdc++fs)

add_executable(forwarder examples/forwarder.cpp)
target_link_libraries(forwarder PRIVATE ixy PRIVATE fmt)

add_executable(generator examples/generator.cpp)
target_link_libraries(generator PRIVATE ixy PRIVATE fmt)
