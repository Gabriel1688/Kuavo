cmake_minimum_required(VERSION 3.12.0)
find_program(CMAKE_C_COMPILER NAMES $ENV{CC} gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} g++ PATHS ENV PATH NO_DEFAULT_PATH)

project(Kuavo)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_ASAN "Build with AddressSanitizer for leak detection" OFF)
if(ENABLE_ASAN)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
endif()

include(FetchContent)
FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.0.tar.gz)
FetchContent_MakeAvailable(spdlog)

SET(THIS_COM "${PROJECT_SOURCE_DIR}/")
set(KuavoConfigPath "${THIS_COM}config/")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(
        Kuavo

        src/Robot.cpp
        src/subsystems/Imu.h
        src/subsystems/Imu.cpp
        src/subsystems/Legged.h
        src/subsystems/Legged.cpp
        src/controllers/Controller.cpp

        src/Test1.hpp
        src/Test1.cpp
        #        src/Mercury_Dyn_environment.cpp
        #        src/Mercury_Dyn_environment.h
)

target_include_directories(
        Kuavo
        PUBLIC
        /usr/local/include/DynaCoRE
        PRIVATE
        include
        src
        lib
        src
)
target_link_options(Kuavo PRIVATE -pg)
target_link_libraries(
        Kuavo
        PUBLIC
        framework
        Mercury_Controller
        dynacore_utils
        rt
)
#add_subdirectory(src/LieGroup)
add_subdirectory(srSimulator)
add_subdirectory(tools)
add_subdirectory(lib)

target_compile_options(Kuavo PRIVATE -Wall -Wextra -Wno-unused-function)