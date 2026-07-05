
#include "RobotBase.h"
#include "common/Config.h"
#include "motor/CANAPI.h"
#include "mqtt/MqttClient.h"
#include "mqtt/Wrapper.h"

namespace hal::init {
extern void InitializeCANAPI(int instanceId);
}

std::shared_ptr<MqttClient> mqClient;

RobotBase::RobotBase() {
    m_threadId = (unsigned long) pthread_self();

//    SetupMathShared();
#if 0//TODO:: check connection with driver station.
    auto inst = nt::NetworkTableInstance::GetDefault();
    // subscribe to "" to force persistent values to propagate to local
    nt::SubscribeMultiple(inst.getHandle(), {{std::string_view{}}});
    if constexpr (!IsSimulation()) {
        inst.StartServer("/home/lvuser/networktables.json");
    } else {
        inst.StartServer();
    }
#endif
    // Call DriverStation::refreshData() to kick things off
    DriverStation::refreshData();
}

//following the procedure in
//https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpilibcExamples/src/main/cpp/examples/HAL/c/Robot.c#L52
void InitializeHAL() {
    client_create();
    mqClient = std::shared_ptr<MqttClient>(g_mqttClient_ptr.load(), [](MqttClient *) {});
    Config::init("../config/config.yaml");
    mqClient->loadConfig("");
    hal::InitializeDriverStation();
    mqClient->start();
    hal::init::InitializeCANAPI(0);// for left leg
    hal::init::InitializeCANAPI(1);// for right leg

    //    InitializeConstants();
    //    InitializeCounter();
    //    InitializeMain();
    //    InitializeNotifier();
    DriverStation::waitForDsConnection(0);
    //hal::WaitForInitialPacket();

    //    m_initialized = true;
    //    m_dashboardDetected = true;
}
/*
 * void WaitForInitialPacket() {
wpi::Event waitForInitEvent;
driverStation->newDataEvents.add(waitForInitEvent.getHandle());
bool timed_out = false;
wpi::waitForObject(waitForInitEvent.getHandle(), 0.1, &timed_out);
// Don't care what the result is, just want to give it a chance.
driverStation->newDataEvents.remove(waitForInitEvent.getHandle());
}
 */