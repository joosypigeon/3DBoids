cmake_minimum_required(VERSION 3.20)

project(boids3d C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(boids
    src/main.c
    src/boids.c
    src/camera.c
    src/normal_random.c
    src/spatial_hash.c
    src/torus.c
)

target_include_directories(boids PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    /usr/local/include
)

target_compile_definitions(boids PRIVATE
    _DEFAULT_SOURCE
    PLATFORM_DESKTOP
    PLATFORM_DESKTOP_GLFW
)

target_compile_options(boids PRIVATE
    -Wall
    -Wextra
    -Wno-missing-braces
    -Wunused-result
    -O2
)

find_package(OpenMP)

if(OpenMP_C_FOUND)
    target_link_libraries(boids PRIVATE OpenMP::OpenMP_C)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(RAYLIB REQUIRED raylib)

target_include_directories(boids PRIVATE
    ${RAYLIB_INCLUDE_DIRS}
)

target_compile_options(boids PRIVATE
    ${RAYLIB_CFLAGS_OTHER}
)

# Prefer the static raylib archive if it exists.
# This avoids depending on libraylib.so at runtime.
find_library(RAYLIB_STATIC
    NAMES libraylib.a raylib
    PATHS
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
)

if(RAYLIB_STATIC)
    message(STATUS "Using static raylib: ${RAYLIB_STATIC}")

    target_link_libraries(boids PRIVATE
        ${RAYLIB_STATIC}
        GL
        m
        pthread
        dl
        rt
        X11
        atomic
    )
else()
    message(WARNING "Static raylib not found; falling back to pkg-config raylib libraries")

    target_link_directories(boids PRIVATE
        ${RAYLIB_LIBRARY_DIRS}
    )

    target_link_libraries(boids PRIVATE
        ${RAYLIB_LIBRARIES}
        GL
        m
        pthread
        dl
        rt
        X11
        atomic
    )
endif()

# Copy runtime assets into the build directory so the program can also be run
# from inside build/ with: ./boids
add_custom_command(TARGET boids POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        $<TARGET_FILE_DIR:boids>/src

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lighting.vs
        $<TARGET_FILE_DIR:boids>/src/lighting.vs

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lighting.fs
        $<TARGET_FILE_DIR:boids>/src/lighting.fs

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/blender_dart.obj
        $<TARGET_FILE_DIR:boids>/blender_dart.obj
)
