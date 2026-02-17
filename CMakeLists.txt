cmake_minimum_required(VERSION 3.10)
set(CMAKE_SKIP_INSTALL_RULES True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(PROJECT_NAME spinach)


project(${PROJECT_NAME})

add_subdirectory(src/physics)
add_subdirectory(src/dev)
add_subdirectory(src/angryBirds)
add_subdirectory(src/unittest)

