cmake_minimum_required(VERSION 3.20)

project(AI_Robot_Arena LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.2.0
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(SDL3)

add_executable(RobotArena src/main.cpp)

target_link_libraries(RobotArena PRIVATE SDL3::SDL3)

add_custom_command(TARGET RobotArena POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:SDL3::SDL3>
        $<TARGET_FILE_DIR:RobotArena>
)


# Download and build Box2D 3.x
FetchContent_Declare(
    box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v3.1.1
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(box2d)

# Link Box2D to our game
target_link_libraries(RobotArena PRIVATE box2d)
