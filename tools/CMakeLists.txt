
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
        ImuSimulator
        imuSimulator.cpp
)

add_executable(
        DamiaoSimulator
        DamiaoSimulator.cpp
)
target_include_directories(
        DamiaoSimulator
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../cmake-build-debug/_deps/spdlog-src/include>
)
target_link_libraries(
        DamiaoSimulator
        PRIVATE
        pthread
        rt
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

add_executable(
        test_controller
        controller.cpp
        mercury_shm.h
)

target_link_libraries(
        test_controller
        PRIVATE
        pthread
        rt
)

add_executable(
        test_actuator
        mercury_shm.h
        actuator.cpp
)

target_link_libraries(
        test_actuator
        PRIVATE
        pthread
        rt
)

add_executable(
        test_controller_v1
        controller_v1.cpp
        mercury_shm_v1.h
)

target_link_libraries(
        test_controller_v1
        PRIVATE
        pthread
        rt
)


add_executable(
        test_controller_v2
        controller_v2.cpp
        mercury_shm_v2.h
)

target_link_libraries(
        test_controller_v2
        PRIVATE
        pthread
        rt
)

add_executable(
        test_actuator_v1
        mercury_shm_v1.h
        actuator_v1.cpp
)

target_link_libraries(
        test_actuator_v1
        PRIVATE
        pthread
        rt
)


add_executable(
        test_actuator_logger
        test_actuator_logger.cpp
        mercury_shm_v2.h
)

target_link_libraries(
        test_actuator_logger
        PRIVATE
        pthread
        rt
        websockets
)

add_executable(
        test_subscriber_influxdb
        mercury_shm_v2.h
        test_subscriber_influxdb.cpp
)

target_link_libraries(
        test_subscriber_influxdb
        PRIVATE
        pthread
        rt
        websockets
        curl
)

add_executable(
        test_scheduling
        scheduling.cpp
)

target_link_libraries(
        test_scheduling
        PRIVATE
        pthread
        rt
)