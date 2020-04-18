cmake_minimum_required(VERSION 3.1)

project(collision)

## If you want to link SFML statically
set(SFML_STATIC_LIBRARIES TRUE)

# set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

## In most cases better set in the CMake cache
set(SFML_DIR "/mingw32/lib/cmake/SFML")

find_package(SFML 2.5 COMPONENTS graphics audio window REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)
set(INCLUDE_PATH "include")
file(GLOB_RECURSE INCLUDES ${INCLUDE_PATH}/*.h)
add_executable(collision ${SOURCES} ${INCLUDES})

target_include_directories(collision PRIVATE "include")
target_link_libraries(collision sfml-graphics sfml-audio)

find_package(ImGui-SFML REQUIRED)
target_link_libraries(collision
	ImGui-SFML::ImGui-SFML
)

# Copy files
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})
file(GLOB BINARY_DEP_DLLS
    "${CMAKE_SOURCE_DIR}/bin/*.dll"
)
file(COPY ${BINARY_DEP_DLLS} DESTINATION ${CMAKE_BINARY_DIR})
