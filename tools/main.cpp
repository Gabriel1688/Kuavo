#include "DriverStationModeThread.h"
#include "ds/DriverStation.h"
#include "ds/GenericHID.h"
#include "ds/XboxController.h"
#include "mqtt/MqttClient.h"
#include "common/Config.h"
#include "mqtt/Wrapper.h"

#include <atomic>
#include <iostream>
std::atomic<bool> m_exit{false};

XboxController xbox_controller{0};
GenericHID joystick{0};

void startCompetition() {
    DriverStationModeThread modeThread;

    wpi::Event event{false, false};
    DriverStation::provideRefreshedDataEventHandle(event.getHandle());
    while (!m_exit) {
        wpi::waitForObject(event.getHandle());
        if (DriverStation::isEnabled()) {
            modeThread.InDisabled(true);
            std::cout << " Robot is Enabled." << std::endl;

            while (DriverStation::isEnabled()) {
                wpi::waitForObject(event.getHandle());

                bool state = joystick.getRawButton(1);
                if (state) {
                    std::cout << "GenericHID joystick button 1 is pressed." << std::endl;
                } else {
                    std::cout << "GenericHID joystick button 1 is released." << std::endl;
                }
                state = xbox_controller.getAButton();
                if (state) {
                    std::cout << "XboxController button AB is pressed." << std::endl;
                } else {
                    std::cout << "XboxController button AB is released." << std::endl;
                }
            }

            modeThread.InDisabled(false);
        }
    }
}

extern "C" {
namespace hal {
void InitializeDriverStation();
}
void InitializeFRCDriverStation();
}
std::shared_ptr<MqttClient> mqClient;
int main() {
    client_create();
    mqClient = std::shared_ptr<MqttClient>(g_mqttClient_ptr.load(), [](MqttClient *) {});
    Config::init("../../config/config.yaml");
    mqClient->loadConfig("");
    mqClient->start();

    InitializeFRCDriverStation();
    hal::InitializeDriverStation();
    DriverStation::refreshData();
    startCompetition();
}