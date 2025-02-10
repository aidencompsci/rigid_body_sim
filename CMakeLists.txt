cmake_minimum_required(VERSION 3.16)
project(cpp_rlimgui_test LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# If building with Emscripten, define PLATFORM_WEB so that raylib is built for WebAssembly.
if(EMSCRIPTEN)
    message(STATUS "Detected Emscripten build; defining PLATFORM_WEB for raylib")
    add_definitions(-DPLATFORM_WEB)
endif()

# Define paths to external libraries
set(RAYLIB_PATH "${CMAKE_SOURCE_DIR}/libs/raylib")
set(IMGUI_PATH "${CMAKE_SOURCE_DIR}/libs/imgui")
set(RLIMGUI_PATH "${CMAKE_SOURCE_DIR}/libs/rlImGui")

# Add raylib subdirectory (its CMakeLists.txt should pick up EMSCRIPTEN and PLATFORM_WEB)
add_subdirectory(${RAYLIB_PATH} EXCLUDE_FROM_ALL)

# Include directories for raylib, ImGui, and rlImGui
include_directories(
    ${RAYLIB_PATH}/src
    ${IMGUI_PATH}
    ${RLIMGUI_PATH}
)

# Collect ImGui source files
set(IMGUI_SOURCES
    ${IMGUI_PATH}/imgui.cpp
    ${IMGUI_PATH}/imgui_demo.cpp
    ${IMGUI_PATH}/imgui_draw.cpp
    ${IMGUI_PATH}/imgui_tables.cpp
    ${IMGUI_PATH}/imgui_widgets.cpp
)

# Collect rlImGui source files
set(RLIMGUI_SOURCES
    ${RLIMGUI_PATH}/rlImGui.cpp
)

# Add the executable, including your main source and the ImGui/rlImGui files
add_executable(${PROJECT_NAME}
    src/main.cpp
    ${IMGUI_SOURCES}
    ${RLIMGUI_SOURCES}
)

# Link raylib into your executable target
target_link_libraries(${PROJECT_NAME} raylib)

# Set the output directory for the executable
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)


if (PLATFORM STREQUAL "Web")
    # Tell Emscripten to build an example.html file.
    set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".html")

    # Required linker flags for using Raylib with Emscripten
    target_link_options(${PROJECT_NAME} PRIVATE
            -sEXPORTED_FUNCTIONS=['_main','_malloc']
            -sEXPORTED_RUNTIME_METHODS=ccall
            -sUSE_GLFW=3
            # --preload-file ${CMAKE_SOURCE_DIR}/resources@/resources
    )
else ()
    # add_custom_command(
    #         TARGET ${PROJECT_NAME} POST_BUILD
    #         COMMAND ${CMAKE_COMMAND} -E copy_directory
    #         ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:${PROJECT_NAME}>/resources
    #         COMMENT "Copying resources to output directory"
    # )
endif()


