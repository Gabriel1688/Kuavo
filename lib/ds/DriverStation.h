#pragma once

#include "DriverStationTypes.h"
#include "common/Synchronization.h"
#include <optional>
#include <string>

typedef int32_t HAL_Bool;

/**
 * Refresh the DS control word.
 *
 * @return true if updated
 */

void InitializeDriverStation();

#ifdef __cplusplus
extern "C" {
#endif

/**
* Gets the current control word of the driver station.
*
* The control word contains the robot state.
*
* @param controlWord the control word (out)
* @return the error code, or 0 for success
*/
int32_t HAL_GetControlWord(HAL_ControlWord *controlWord);

/**
 * Gets the axes of a specific joystick.
 *
 * @param joystickNum the joystick number
 * @param axes        the axes values (output)
 * @return the error code, or 0 for success
 */
int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes *axes);

/**
 * Gets the POVs of a specific joystick.
 *
 * @param joystickNum the joystick number
 * @param povs        the pov values (output)
 * @return the error code, or 0 for success
 */
int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs *povs);

/**
 * Gets the buttons of a specific joystick.
 *
 * @param joystickNum the joystick number
 * @param buttons     the button values (output)
 * @return the error code, or 0 for success
 */
int32_t HAL_GetJoystickButtons(int32_t joystickNum,
                               HAL_JoystickButtons *buttons);

void HAL_GetAllJoystickData(HAL_JoystickAxes *axes, HAL_JoystickPOVs *povs,
                            HAL_JoystickButtons *buttons);

/**
 * Refresh the DS control word.
 *
 * @return true if updated
 */
HAL_Bool HAL_RefreshDSData(void);

/**
 * Adds an event handle to be signalled when new data arrives.
 *
 * @param handle the event handle to be signalled
 */
void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle);

/**
 * Removes the event handle from being signalled when new data arrives.
 *
 * @param handle the event handle to remove
 */
void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle);

#ifdef __cplusplus
}// extern "C"
#endif
/** @} */

/**
 * Provide access to the network communication data to / from the Driver Station.
 */
class DriverStation final {
public:
    /**
       * The type of robot match that the robot is part of.
       */
    enum MatchType {
        /// None.
        kNone,
        /// Practice.
        kPractice,
        /// Qualification.
        kQualification,
        /// Elimination.
        kElimination
    };

    /// Number of Joystick ports.
    static constexpr int kJoystickPorts = 6;

    /**
      * The state of one joystick button. %Button indexes begin at 1.
      *
      * @param stick  The joystick to read.
      * @param button The button index, beginning at 1.
      * @return The state of the joystick button.
      */
    static bool getStickButton(int stick, int button);

    /**
      * Whether one joystick button was pressed since the last check. %Button
      * indexes begin at 1.
      *
      * @param stick  The joystick to read.
      * @param button The button index, beginning at 1.
      * @return Whether the joystick button was pressed since the last check.
      */
    static bool getStickButtonPressed(int stick, int button);

    /**
      * Whether one joystick button was released since the last check. %Button
      * indexes begin at 1.
      *
      * @param stick  The joystick to read.
      * @param button The button index, beginning at 1.
      * @return Whether the joystick button was released since the last check.
      */
    static bool getStickButtonReleased(int stick, int button);

    /**
      * Get the value of the axis on a joystick.
      *
      * This depends on the mapping of the joystick connected to the specified
      * port.
      *
      * @param stick The joystick to read.
      * @param axis  The analog axis value to read from the joystick.
      * @return The value of the axis on the joystick.
      */
    static double getStickAxis(int stick, int axis);

    /**
      * Get the state of a pov on the joystick.
      *
      * @return the angle of the pov in degrees, or -1 if the pov is not pressed.
      */
    static int getStickPOV(int stick, int pov);

    /**
      * The state of the buttons on the joystick.
      *
      * @param stick The joystick to read.
      * @return The state of the buttons on the joystick.
      */
    static int getStickButtons(int stick);

    /**
      * Returns the number of axes on a given joystick port.
      *
      * @param stick The joystick port number
      * @return The number of axes on the indicated joystick
      */
    static int getStickAxisCount(int stick);

    /**
      * Returns the number of POVs on a given joystick port.
      *
      * @param stick The joystick port number
      * @return The number of POVs on the indicated joystick
      */
    static int getStickPOVCount(int stick);

    /**
      * Returns the number of buttons on a given joystick port.
      *
      * @param stick The joystick port number
      * @return The number of buttons on the indicated joystick
      */
    static int getStickButtonCount(int stick);

    /**
      * Returns the types of Axes on a given joystick port.
      *
      * @param stick The joystick port number and the target axis
      * @param axis  The analog axis value to read from the joystick.
      * @return What type of axis the axis is reporting to be
      */
    static int getJoystickAxisType(int stick, int axis);

    /**
      * Returns if a joystick is connected to the Driver Station.
      *
      * This makes a best effort guess by looking at the reported number of axis,
      * buttons, and POVs attached.
      *
      * @param stick The joystick port number
      * @return true if a joystick is connected
      */
    static bool isJoystickConnected(int stick);

    /**
      * Check if the DS has enabled the robot.
      *
      * @return True if the robot is enabled and the DS is connected
      */
    static bool isEnabled();

    /**
      * Check if the robot is disabled.
      *
      * @return True if the robot is explicitly disabled or the DS is not connected
      */
    static bool isDisabled();

    /**
      * Check if the robot is e-stopped.
      *
      * @return True if the robot is e-stopped
      */
    static bool isEStopped();

    /**
      * Check if the DS is commanding autonomous mode.
      *
      * @return True if the robot is being commanded to be in autonomous mode
      */
    static bool isAutonomous();

    /**
      * Check if the DS is commanding autonomous mode and if it has enabled the
      * robot.
      *
      * @return True if the robot is being commanded to be in autonomous mode and
      * enabled.
      */
    static bool isAutonomousEnabled();

    /**
      * Check if the DS is commanding teleop mode.
      *
      * @return True if the robot is being commanded to be in teleop mode
      */
    static bool isTeleop();

    /**
      * Check if the DS is commanding teleop mode and if it has enabled the robot.
      *
      * @return True if the robot is being commanded to be in teleop mode and
      * enabled.
      */
    static bool isTeleopEnabled();

    /**
       * Check if the DS is commanding test mode.
       *
       * @return True if the robot is being commanded to be in test mode
       */
    static bool isTest();

    /**
      * Check if the DS is commanding Test mode and if it has enabled the robot.
      *
      * @return True if the robot is being commanded to be in Test mode and
      * enabled.
      */
    static bool isTestEnabled();

    /**
      * Check if the DS is attached.
      *
      * @return True if the DS is connected to the robot
      */
    static bool isDSAttached();

    /**
      * Wait for a DS connection.
      *
      * @param timeout timeout in seconds. 0 for infinite.
      * @return true if connected, false if timeout
      */
    static bool waitForDsConnection(int timeout);

    /**
      * Read the battery voltage.
      *
      * @return The battery voltage in Volts.
      */
    static double getBatteryVoltage();

    /**
      * Copy data from the DS task for the user. If no new data exists, it will
      * just be returned, otherwise the data will be copied from the DS polling
      * loop.
    */
    static void refreshData();

    /**
      * Registers the given handle for DS data refresh notifications.
      *
      * @param handle The event handle.
      */
    static void provideRefreshedDataEventHandle(WPI_EventHandle handle);

    /**
      * Unregisters the given handle from DS data refresh notifications.
      *
      * @param handle The event handle.
      */
    static void removeRefreshedDataEventHandle(WPI_EventHandle handle);

private:
    DriverStation() = default;
};
