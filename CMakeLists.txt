cmake_minimum_required(VERSION 3.10)

project(SAT_COLLISION)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug) # Debug Release

# Add executable
add_executable(SAT_COLLISION
    src/Box.cpp
    src/main.cpp
    src/vector_math.cpp
)
target_include_directories(SAT_COLLISION
    PRIVATE ${PROJECT_SOURCE_DIR}/include
)

if(APPLE)
    set(SFML_STATIC_LIBRARIES False)
    set(SFML_DIR "/usr/local/Cellar/sfml/2.5.1")
    target_compile_definitions(SAT_COLLISION PUBLIC "GL_SILENCE_DEPRECATION")
elseif(MSYS)
    set(SFML_STATIC_LIBRARIES True)
    set(SFML_DIR "/mingw32/lib/cmake/SFML")
else()
    message(FATAL_ERROR "Not supported")
endif()


# Add SFML
find_package(SFML 2.5 COMPONENTS system window graphics network audio REQUIRED)
target_link_libraries(SAT_COLLISION
    sfml-system sfml-window sfml-graphics sfml-network sfml-audio
)

# Add ImGui-SFML
find_package(ImGui-SFML REQUIRED)
if(NOT ImGui-SFML_FOUND)
    message(FATAL_ERROR "ImGui-SFML not found")
endif()
target_link_libraries(SAT_COLLISION
    ImGui-SFML::ImGui-SFML
)

# Add OpenGL
find_package(OpenGL REQUIRED)
if(NOT OpenGL_FOUND)
    message(FATAL_ERROR "OpenGL not found")
endif()
target_include_directories(SAT_COLLISION
    PUBLIC ${OPENGL_INCLUDE_DIRS}
)
target_link_libraries(SAT_COLLISION
    ${OPENGL_LIBRARIES}
)

# Add fmt
find_package(fmt REQUIRED)
if(NOT fmt_FOUND)
    message(FATAL_ERROR "fmt not found")
endif()
target_link_libraries(SAT_COLLISION
    fmt::fmt
)

if(MSYS)
    target_link_libraries(SAT_COLLISION
        # -static-libgcc -static-libstdc++ -lwsock32 -lws2_32
        # -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive
        # -mconsole / -mwindows
    )
endif()
