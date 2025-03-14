cmake_minimum_required(VERSION 3.30)
project(Orbits)

set(CMAKE_CXX_STANDARD 26)

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
        raylib
        GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(raylib)

# Gather source files from the src folder
file(GLOB_RECURSE SRC_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

# Specify source files in the project root (e.g., main.cpp, game.cpp)
set(ROOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Body.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Sim.cpp"
)

# Define the executable target using all gathered sources
add_executable(Orbits ${ROOT_SOURCES} ${SRC_SOURCES} )

# Set include directories for the project (both the root and src folders)
target_include_directories(Orbits PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/"
        "${CMAKE_CURRENT_LIST_DIR}/src/"
)

# Link against raylib
target_link_libraries(Orbits PRIVATE raylib)

# Define compile definitions with assets path
target_compile_definitions(Orbits PUBLIC ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/")