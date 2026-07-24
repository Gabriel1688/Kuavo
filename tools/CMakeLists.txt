
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

add_executable(
        mercury_controller
        mercury_controller.cpp
        ../include/mercury_shm.h
)

target_link_libraries(
        mercury_controller
        PRIVATE
        pthread
        rt
)

add_executable(
        test_actuator_logger
        test_actuator_logger.cpp
        ../include/mercury_shm.h
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
        ../include/mercury_shm.h
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

add_executable(
        test_shm_lifecycle
        test_shm_lifecycle.cpp
        ../include/mercury_shm.h
)

target_link_libraries(
        test_shm_lifecycle
        PRIVATE
        pthread
        rt
)