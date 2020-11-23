cmake_minimum_required(VERSION 3.10)

project(Physcis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug) # Debug Release

# enable doxygen
# option(ENABLE_DOXYGEN "Enable doxygen doc builds of source" ON)
# if(ENABLE_DOXYGEN)
#     add_subdirectory(docs)
# endif()


# Add executable
add_executable(Physics
    src/main.cpp
    src/Box.cpp
    src/Shape.cpp
    src/Polygon.cpp
    src/Circle.cpp
    src/Physics.cpp
    src/vector_math.cpp
    src/World.cpp
    src/Helper.cpp
    src/Arbiter.cpp
    src/Joint.cpp include/Collision.h)

target_include_directories(Physics
    PRIVATE ${PROJECT_SOURCE_DIR}/include
)

set(SFML_STATIC_LIBRARIES False)
set(SFML_DIR "/mingw32/lib/cmake/SFML")

# Add SFML
find_package(SFML 2.5 COMPONENTS system window graphics network audio REQUIRED)
target_link_libraries(Physics
    sfml-system sfml-window sfml-graphics sfml-network sfml-audio
)

# Add ImGui-SFML
find_package(ImGui-SFML REQUIRED)
if(NOT ImGui-SFML_FOUND)
    message(FATAL_ERROR "ImGui-SFML not found")
endif()
target_link_libraries(Physics
    ImGui-SFML::ImGui-SFML
)

# Add OpenGL
find_package(OpenGL REQUIRED)
if(NOT OpenGL_FOUND)
    message(FATAL_ERROR "OpenGL not found")
endif()
target_include_directories(Physics
    PUBLIC ${OPENGL_INCLUDE_DIRS}
)
target_link_libraries(Physics
    ${OPENGL_LIBRARIES}
)

# Add fmt
find_package(fmt REQUIRED)
if(NOT fmt_FOUND)
    message(FATAL_ERROR "fmt not found")
endif()
target_link_libraries(Physics
    fmt::fmt
)
