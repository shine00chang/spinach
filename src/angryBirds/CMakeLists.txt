set(EXE_NAME angrybirds)
set(LIB_NAME engine)

find_package(SDL2 REQUIRED)

add_executable(${EXE_NAME} env.cpp ../main.cpp)
target_include_directories(${EXE_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
target_link_libraries(${EXE_NAME} SDL2::SDL2 ${LIB_NAME})

set_target_properties(${LIB_NAME} ${EXE_NAME} PROPERTIES CXX_STANDARD 20)
