set(LIB_NAME physicsLib)
set(EXE_NAME unit)


find_package(SDL2 REQUIRED)
add_executable(${EXE_NAME} envs.cpp ../main.cpp)
target_include_directories(${EXE_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/physics)
target_link_libraries(${EXE_NAME} ${LIB_NAME} SDL2::SDL2)

set_target_properties(${LIB_NAME} ${EXE_NAME} PROPERTIES CXX_STANDARD 20)
