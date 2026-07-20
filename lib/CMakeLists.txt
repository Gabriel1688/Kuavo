set(CMAKE_CXX_STANDARD 20)
cmake_policy(SET CMP0077 NEW)
set(LWS_WITH_CLIENT 1)
set(LWS_ROLE_MQTT 1)
set(LWS_WITH_NETWORK 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs  -fPIC -g -O0")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

include(FetchContent)
FetchContent_Declare(libwebsockets URL https://github.com/warmcat/libwebsockets/archive/refs/tags/v4.5.8.tar.gz)
set(LWS_WITHOUT_TESTAPPS 1)
FetchContent_MakeAvailable(libwebsockets)
message(CHECK_START "Fetching Eigen3")

FetchContent_Declare(Eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_TESTING OFF)
set(EIGEN_MPL2_ONLY ON)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_BUILD_DOC OFF)
FetchContent_MakeAvailable(Eigen)
message(CHECK_PASS "fetched")

set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(framework
        common/TimerManager.cpp
        common/Synchronization.cpp
        common/Notifier.cpp
        common/FdEvent.cpp
        common/Config.cpp

        ds/DSControlWord.cpp
        ds/FRCDriverStation.cpp
        ds/BooleanEvent.cpp
        ds/DriverStation.cpp
        ds/GenericHID.cpp
        ds/XboxController.cpp
        ds/EventLoop.cpp

        robot/RobotBase.cpp
        robot/IterativeRobotBase.cpp
        robot/TimedRobot.cpp
        robot/SubsystemBase.cpp

        motor/CAN.cpp
        motor/CANAPI.cpp
        motor/Motor.cpp
        motor/UdpServer.cpp

        telemetry/DataLog.cpp
        telemetry/RobotStatus.cpp
        mqtt/Wrapper.cpp
        mqtt/MqttClient.cpp
        imu/ImuReader.cpp
        composer/Composer.cpp
        utility/Utility.cpp
)
set_target_properties(framework PROPERTIES DEBUG_POSTFIX "d")

target_include_directories(framework
        PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/lib
        /usr/local/include/DynaCoRE
        PUBLIC)

target_link_libraries(
        framework
        PUBLIC
        websockets
        spdlog
        Eigen3::Eigen
        dynacore_yaml-cpp
        nlohmann_json::nlohmann_json
)

target_compile_options(framework PRIVATE -Wall -Wextra -Wno-unused-function)
