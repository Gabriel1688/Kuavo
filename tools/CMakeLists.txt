
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIE")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -pie")
add_executable(
        DriverStation
        DriverStationModeThread.cpp
        main.cpp
)

add_executable(
        PackageGenerator
        PackageGenerator.cpp
)

add_executable(
        UdpClient
        UdpClient.cpp
)
target_include_directories(
        UdpClient
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../cmake-build-debug/_deps/spdlog-src/include>
)

target_include_directories(
        DriverStation
        PRIVATE
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
        DriverStation
        PRIVATE
        framework
)

target_link_libraries(
        UdpClient
        PRIVATE
        spdlog
)
