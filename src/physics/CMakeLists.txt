file(GLOB SOURCES *.cpp)

set(LIB_NAME physicsLib)

add_library(${LIB_NAME} STATIC ${SOURCES})
target_include_directories(${LIB_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/physics)


find_package(SDL2 REQUIRED)
target_include_directories(${LIB_NAME} PUBLIC ${SDL2_INCLUDE_DIRS})

set_target_properties(${LIB_NAME} ${EXE_NAME} PROPERTIES CXX_STANDARD 20)


