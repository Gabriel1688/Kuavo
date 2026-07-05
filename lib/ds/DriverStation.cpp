#include "DriverStation.h"
#include "DriverStationTypes.h"
#include "EventVector.h"
#include <array>
#include <string>

#include <mutex>

extern "C" {
namespace hal {
extern void InitializeDriverStation();
}
HAL_Bool HAL_RefreshDSData(void);
}

struct Instance {
    Instance();

    ~Instance();

    EventVector refreshEvents;

    // Joystick button rising/falling edge flags
    std::mutex buttonEdgeMutex;
    std::array<HAL_JoystickButtons, DriverStation::kJoystickPorts> previousButtonStates;
    std::array<uint32_t, DriverStation::kJoystickPorts> joystickButtonsPressed;
    std::array<uint32_t, DriverStation::kJoystickPorts> joystickButtonsReleased;

    // Robot state status variables
    bool userInDisabled = false;
    bool userInAutonomous = false;
    bool userInTeleop = false;
    bool userInTest = false;
    int nextMessageTime = 0;//0_s;
};

static constexpr auto kJoystickUnpluggedMessageInterval = 1;//1_s;

static Instance &GetInstance() {
    static Instance instance;
    return instance;
}

Instance::Instance() {
    //    HAL_Initialize(500, 0);
    //    InitializeHAL();
    //hal::RestartTiming();
    hal::InitializeDriverStation();

    // All joysticks should default to having zero axes, povs and buttons, so
    // uninitialized memory doesn't get sent to motor controllers.
    for (unsigned int i = 0; i < DriverStation::kJoystickPorts; i++) {
        joystickButtonsPressed[i] = 0;
        joystickButtonsReleased[i] = 0;
        previousButtonStates[i].count = 0;
        previousButtonStates[i].buttons = 0;
    }
}

Instance::~Instance() {
}

extern int32_t HAL_GetJoystickButtonsInternal(int32_t joystickNum, HAL_JoystickButtons *buttons);

bool DriverStation::getStickButton(int stick, int button) {
    if (stick < 0 || stick >= kJoystickPorts || button < 1) {
        return false;
    }

    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);

    return (buttons.buttons & (1 << (button - 1))) != 0;
}

bool DriverStation::getStickButtonPressed(int stick, int button) {

    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);

    auto &inst = ::GetInstance();
    std::unique_lock lock(inst.buttonEdgeMutex);
    // If button was pressed, clear flag and return true
    if (inst.joystickButtonsPressed[stick] & 1 << (button - 1)) {
        inst.joystickButtonsPressed[stick] &= ~(1 << (button - 1));
        return true;
    }
    return false;
}

bool DriverStation::getStickButtonReleased(int stick, int button) {
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);

    auto &inst = ::GetInstance();
    std::unique_lock lock(inst.buttonEdgeMutex);
    // If button was released, clear flag and return true
    if (inst.joystickButtonsReleased[stick] & 1 << (button - 1)) {
        inst.joystickButtonsReleased[stick] &= ~(1 << (button - 1));
        return true;
    }
    return false;
}

double DriverStation::getStickAxis(int stick, int axis) {
    HAL_JoystickAxes axes;
    HAL_GetJoystickAxes(stick, &axes);
    return axes.axes[axis];
}

int DriverStation::getStickPOV(int stick, int pov) {
    HAL_JoystickPOVs povs;
    HAL_GetJoystickPOVs(stick, &povs);
    return povs.povs[pov];
}

int DriverStation::getStickButtons(int stick) {
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);
    return buttons.buttons;
}

int DriverStation::getStickAxisCount(int stick) {
    HAL_JoystickAxes axes;
    HAL_GetJoystickAxes(stick, &axes);
    return axes.count;
}

int DriverStation::getStickPOVCount(int stick) {
    HAL_JoystickPOVs povs;
    HAL_GetJoystickPOVs(stick, &povs);
    return povs.count;
}

int DriverStation::getStickButtonCount(int stick) {
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);
    return buttons.count;
}

bool DriverStation::isJoystickConnected(int stick) {
    return getStickAxisCount(stick) > 0 || getStickButtonCount(stick) > 0 || getStickPOVCount(stick) > 0;
}

bool DriverStation::isEnabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.enabled && controlWord.dsAttached;
}

bool DriverStation::isDisabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return !(controlWord.enabled && controlWord.dsAttached);
}

bool DriverStation::isEStopped() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.eStop;
}

bool DriverStation::isAutonomous() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.autonomous;
}

bool DriverStation::isAutonomousEnabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.autonomous && controlWord.enabled;
}

bool DriverStation::isTeleop() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return !(controlWord.autonomous || controlWord.test);
}

bool DriverStation::isTeleopEnabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return !controlWord.autonomous && !controlWord.test && controlWord.enabled;
}

bool DriverStation::isDSAttached() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.dsAttached;
}

// std::string DriverStation::GetEventName() {
//    HAL_MatchInfo info;
//    HAL_GetMatchInfo(&info);
//    return info.eventName;
//}

bool DriverStation::waitForDsConnection(int timeout) {
    bool result = false;
    wpi::Event event{true, false};
    HAL_ProvideNewDataEventHandle(event.getHandle());
    if (timeout == 0) {
        result = wpi::waitForObject(event.getHandle());
    } else {
        result = wpi::waitForObject(event.getHandle(), timeout, nullptr);
    }

    HAL_RemoveNewDataEventHandle(event.getHandle());
    refreshData();
    return result;
}

double DriverStation::getBatteryVoltage() {
    int32_t status = 0;
    double voltage = 0.0;
    //    double voltage = HAL_GetVinVoltage(&status);
    //    FRC_CheckErrorStatus(status, "getVinVoltage");

    return voltage;
}

/**
 * Copy data from the DS task for the user.
 *
 * If no new data exists, it will just be returned, otherwise
 * the data will be copied from the DS polling loop.
 */
void DriverStation::refreshData() {
    HAL_RefreshDSData();
    auto &inst = ::GetInstance();
    {
        // Compute the pressed and released buttons
        HAL_JoystickButtons currentButtons;
        std::unique_lock lock(inst.buttonEdgeMutex);

        for (int32_t i = 0; i < DriverStation::kJoystickPorts; i++) {
            HAL_GetJoystickButtons(i, &currentButtons);

            // If buttons weren't pressed and are now, set flags in m_buttonsPressed
            inst.joystickButtonsPressed[i] |=
                ~inst.previousButtonStates[i].buttons & currentButtons.buttons;

            // If buttons were pressed and aren't now, set flags in m_buttonsReleased
            inst.joystickButtonsReleased[i] |=
                inst.previousButtonStates[i].buttons & ~currentButtons.buttons;

            inst.previousButtonStates[i] = currentButtons;
        }
    }

    inst.refreshEvents.wakeup();
}

void DriverStation::provideRefreshedDataEventHandle(WPI_EventHandle handle) {
    auto &inst = ::GetInstance();
    inst.refreshEvents.add(handle);
}

void DriverStation::removeRefreshedDataEventHandle(WPI_EventHandle handle) {
    auto &inst = ::GetInstance();
    inst.refreshEvents.remove(handle);
}
