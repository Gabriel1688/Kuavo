#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>

/*
 * Enums matching the DS library
 */
enum DS_ControlMode { DS_CONTROL_TELEOPERATED,
                      DS_CONTROL_TEST,
                      DS_CONTROL_AUTONOMOUS };
enum DS_Alliance { DS_ALLIANCE_RED,
                   DS_ALLIANCE_BLUE };
enum DS_Position { DS_POSITION_1,
                   DS_POSITION_2,
                   DS_POSITION_3 };

/*
 * Protocol bytes
 *
 * Control code bits match the HAL_ControlWord bitfield layout so that the
 * receiver can memcpy the wire byte directly into the struct:
 *   bit 0 = enabled
 *   bit 1 = autonomous
 *   bit 2 = test
 *   bit 3 = eStop
 *   bit 4 = fmsAttached
 *   bit 5 = dsAttached
 */
static const uint8_t cRequestRestartCode = 0x04;
static const uint8_t cRequestReboot = 0x08;
static const uint8_t cRequestNormal = 0x00;
static const uint8_t cTagCommVersion = 0x01;
static const uint8_t cTeleoperated = 0x00;   /* neither autonomous nor test */
static const uint8_t cTest = 0x04;           /* HAL_ControlWord bit 2 */
static const uint8_t cAutonomous = 0x02;     /* HAL_ControlWord bit 1 */
static const uint8_t cEnabled = 0x01;        /* HAL_ControlWord bit 0 */
static const uint8_t cEmergencyStop = 0x08;  /* HAL_ControlWord bit 3 */
static const uint8_t cFMSConnected = 0x10;   /* HAL_ControlWord bit 4 */
static const uint8_t cDSAttached = 0x20;     /* HAL_ControlWord bit 5 */
static const uint8_t cTagDate = 0x0f;
static const uint8_t cTagJoystick = 0x0c;
static const uint8_t cTagTimezone = 0x10;
static const uint8_t cRed1 = 0x00;
static const uint8_t cRed2 = 0x01;
static const uint8_t cRed3 = 0x02;
static const uint8_t cBlue1 = 0x03;
static const uint8_t cBlue2 = 0x04;
static const uint8_t cBlue3 = 0x05;

/*
 * Configurable test state
 */
static DS_ControlMode g_control_mode = DS_CONTROL_TELEOPERATED;
static DS_Alliance g_alliance = DS_ALLIANCE_RED;
static DS_Position g_position = DS_POSITION_1;
static int g_robot_enabled = 0;
static int g_estop = 0;
static int g_fms_comms = 0;
static int g_robot_comms = 0;

static unsigned int send_time_data = 0;
static unsigned int sent_robot_packets = 0;
static int reboot = 0;
static int restart_code = 0;

/*
 * Joystick test data
 */
struct Joystick {
    int num_axes;
    float axes[6];
    int num_buttons;
    int buttons[16];
    int num_hats;
    int hats[4];
};

static Joystick g_joysticks[4];
static int g_joystick_count = 0;

/*
 * Stub functions for the DS config / joystick API
 */
static DS_ControlMode cfgGetControlMode(void) { return g_control_mode; }
static DS_Alliance cfgGetAlliance(void) { return g_alliance; }
static DS_Position cfgGetPosition(void) { return g_position; }
static int cfgGetRobotEnabled(void) { return g_robot_enabled; }
static int cfgGetEmergencyStopped(void) { return g_estop; }
static int cfgGetFMSCommunications(void) { return g_fms_comms; }
static int cfgGetRobotCommunications(void) { return g_robot_comms; }

static int dsGetJoystickCount(void) { return g_joystick_count; }
static int dsGetJoystickNumAxes(int i) { return g_joysticks[i].num_axes; }
static float dsGetJoystickAxis(int i, int a) { return g_joysticks[i].axes[a]; }
static int dsGetJoystickNumButtons(int i) { return g_joysticks[i].num_buttons; }
static int dsGetJoystickButton(int i, int b) { return g_joysticks[i].buttons[b]; }
static int dsGetJoystickNumHats(int i) { return g_joysticks[i].num_hats; }
static int dsGetJoystickHat(int i, int h) { return g_joysticks[i].hats[h]; }
static uint8_t dsFloatToByte(float val, int) { return (uint8_t) ((val + 1.0f) / 2.0f * 255.0f); }

static std::vector<uint8_t> create_robot_packet(void) {
    std::vector<uint8_t> data(6, 0);

    /* Add packet index */
    data[0] = (sent_robot_packets >> 8);
    data[1] = (sent_robot_packets);

    /* Add packet header */
    data[2] = cTagCommVersion;

    /* Add control code (matches HAL_ControlWord bitfield layout) */
    {
        uint8_t code = cDSAttached;  /* always set – we are the DS */

        switch (cfgGetControlMode()) {
        case DS_CONTROL_TEST: code |= cTest; break;
        case DS_CONTROL_AUTONOMOUS: code |= cAutonomous; break;
        case DS_CONTROL_TELEOPERATED: code |= cTeleoperated; break;
        default: break;
        }

        if (cfgGetFMSCommunications())
            code |= cFMSConnected;
        if (cfgGetEmergencyStopped())
            code |= cEmergencyStop;
        if (cfgGetRobotEnabled())
            code |= cEnabled;

        data[3] = code;
    }

    /* Add request code */
    {
        uint8_t code = cRequestNormal;

        if (cfgGetRobotCommunications()) {
            if (reboot)
                code = cRequestReboot;
            else if (restart_code)
                code = cRequestRestartCode;
        }

        data[4] = code;
    }

    /* Add team station */
    {
        uint8_t station = cRed1;

        if (cfgGetPosition() == DS_POSITION_1) {
            station = (cfgGetAlliance() == DS_ALLIANCE_RED) ? cRed1 : cBlue1;
        } else if (cfgGetPosition() == DS_POSITION_2) {
            station = (cfgGetAlliance() == DS_ALLIANCE_RED) ? cRed2 : cBlue2;
        } else if (cfgGetPosition() == DS_POSITION_3) {
            station = (cfgGetAlliance() == DS_ALLIANCE_RED) ? cRed3 : cBlue3;
        }

        data[5] = station;
    }

    /* Add timezone data (if robot wants it) */
    if (send_time_data) {
        std::vector<uint8_t> tz_data(13, 0);

        time_t rt = 0;
        uint32_t ms = 0;
        struct tm timeinfo;

        localtime_r(&rt, &timeinfo);
        const char *tz_cstr = timeinfo.tm_zone;
        std::vector<uint8_t> tz(tz_cstr, tz_cstr + strlen(tz_cstr));
        tz_data[0] = (uint8_t) cTagDate;
        tz_data[1] = (uint8_t) (ms >> 24);
        tz_data[2] = (uint8_t) (ms >> 16);
        tz_data[3] = (uint8_t) (ms >> 8);
        tz_data[4] = (uint8_t) (ms);
        tz_data[5] = (uint8_t) timeinfo.tm_sec;
        tz_data[6] = (uint8_t) timeinfo.tm_min;
        tz_data[7] = (uint8_t) timeinfo.tm_hour;
        tz_data[8] = (uint8_t) timeinfo.tm_yday;
        tz_data[9] = (uint8_t) timeinfo.tm_mon;
        tz_data[10] = (uint8_t) timeinfo.tm_year;

        tz_data[11] = cTagTimezone;
        tz_data[12] = (uint8_t) tz.size();

        //        tz_data.insert(tz_data.end(), tz.begin(), tz.end());
        data.insert(data.end(), tz_data.begin(), tz_data.end());
    }

    /* Add joystick data */
    if (sent_robot_packets > 5) {
        int i = 0;
        int j = 0;
        std::vector<uint8_t> js_data;

        for (i = 0; i < dsGetJoystickCount(); ++i) {
            /* Joystick size */
            {
                int header_size = 2;
                int button_data = dsGetJoystickNumButtons(i) + 1;
                int axis_data = dsGetJoystickNumAxes(i) + 1;
                int hat_data = (dsGetJoystickNumHats(i) * 2) + 1;
                js_data.push_back(header_size + button_data + axis_data + hat_data);
            }
            js_data.push_back(cTagJoystick);

            /* Axis data */
            js_data.push_back(dsGetJoystickNumAxes(i));
            for (j = 0; j < dsGetJoystickNumAxes(i); ++j)
                js_data.push_back(dsFloatToByte(dsGetJoystickAxis(i, j), 1));

            /* Button data */
            uint16_t button_flags = 0;
            for (j = 0; j < dsGetJoystickNumButtons(i); ++j)
                button_flags += dsGetJoystickButton(i, j) ? (int) pow(2, j) : 0;

            js_data.push_back(dsGetJoystickNumButtons(i));
            js_data.push_back((uint8_t) (button_flags >> 8));
            js_data.push_back((uint8_t) (button_flags));

            /* Hat data */
            js_data.push_back(dsGetJoystickNumHats(i));
            for (j = 0; j < dsGetJoystickNumHats(i); ++j) {
                js_data.push_back((uint8_t) (dsGetJoystickHat(i, j) >> 8));
                js_data.push_back((uint8_t) (dsGetJoystickHat(i, j)));
            }
        }

        data.insert(data.end(), js_data.begin(), js_data.end());
    }

    ++sent_robot_packets;
    return data;
}

/* =========================================================================
 * Helper: hex-dump a packet
 * ========================================================================= */
static void printPacket(const char *label, const std::vector<uint8_t> &pkt) {
    printf("--- %s (size=%zu) ---\n", label, pkt.size());
    for (size_t i = 0; i < pkt.size(); ++i) {
        printf(" %02X", pkt[i]);
        if ((i + 1) % 16 == 0)
            printf("\n");
    }
    if (pkt.size() % 16 != 0)
        printf("\n");
    printf("\n");
}

/* =========================================================================
 * Test scenarios
 * ========================================================================= */
int main(void) {
    /* ---- Test 1: minimal packet (first 6 packets have no joystick data) ---- */
    printf("=== Test 1: Teleop, disabled, Red-1, packet#0, no joystick ===\n");
    sent_robot_packets = 0;
    send_time_data = 0;
    g_control_mode = DS_CONTROL_TELEOPERATED;
    g_robot_enabled = 0;
    g_estop = 0;
    g_fms_comms = 0;
    g_robot_comms = 0;
    g_alliance = DS_ALLIANCE_RED;
    g_position = DS_POSITION_1;
    reboot = 0;
    restart_code = 0;
    g_joystick_count = 0;

    std::vector<uint8_t> pkt1 = create_robot_packet();
    printPacket("Test 1", pkt1);

    /* ---- Test 2: enabled, autonomous, Blue-2, FMS connected ---- */
    printf("=== Test 2: Auto, enabled, Blue-2, FMS connected, packet#1 ===\n");
    g_control_mode = DS_CONTROL_AUTONOMOUS;
    g_robot_enabled = 1;
    g_fms_comms = 1;
    g_alliance = DS_ALLIANCE_BLUE;
    g_position = DS_POSITION_2;

    std::vector<uint8_t> pkt2 = create_robot_packet();
    printPacket("Test 2", pkt2);

    /* ---- Test 3: with timezone data ---- */
    printf("=== Test 3: Test mode, e-stop, Red-3, with timezone, packet#2 ===\n");
    g_control_mode = DS_CONTROL_TEST;
    g_estop = 1;
    g_robot_enabled = 0;
    g_fms_comms = 0;
    g_alliance = DS_ALLIANCE_RED;
    g_position = DS_POSITION_3;
    send_time_data = 1;

    std::vector<uint8_t> pkt3 = create_robot_packet();
    printPacket("Test 3 (with timezone)", pkt3);

    /* ---- Test 4: with reboot request and timezone ---- */
    printf("=== Test 4: Teleop, reboot request, robot comms, packet#3 ===\n");
    g_control_mode = DS_CONTROL_TELEOPERATED;
    g_estop = 0;
    g_robot_enabled = 1;
    g_robot_comms = 1;
    g_alliance = DS_ALLIANCE_RED;
    g_position = DS_POSITION_1;
    reboot = 1;
    send_time_data = 1;

    std::vector<uint8_t> pkt4 = create_robot_packet();
    printPacket("Test 4 (reboot)", pkt4);
    reboot = 0;

    /* ---- Advance counter past 5 so joystick data is included ---- */
    sent_robot_packets = 6;

    /* ---- Test 5: with 1 joystick ---- */
    printf("=== Test 5: Teleop, enabled, Blue-1, 1 joystick, packet#6 ===\n");
    g_control_mode = DS_CONTROL_TELEOPERATED;
    g_robot_enabled = 1;
    g_robot_comms = 1;
    g_fms_comms = 0;
    g_alliance = DS_ALLIANCE_BLUE;
    g_position = DS_POSITION_1;
    send_time_data = 1;

    g_joystick_count = 1;
    g_joysticks[0].num_axes = 2;
    g_joysticks[0].axes[0] = -1.0f; /* full left  */
    g_joysticks[0].axes[1] = 0.0f;  /* center     */
    g_joysticks[0].num_buttons = 4;
    g_joysticks[0].buttons[0] = 1; /* pressed    */
    g_joysticks[0].buttons[1] = 0;
    g_joysticks[0].buttons[2] = 1; /* pressed    */
    g_joysticks[0].buttons[3] = 0;
    g_joysticks[0].num_hats = 1;
    g_joysticks[0].hats[0] = 90;

    std::vector<uint8_t> pkt5 = create_robot_packet();
    printPacket("Test 5 (1 joystick)", pkt5);
    /* Write test 5 packet to binary file */
    {
        FILE *fp = fopen("button_pressed.bin", "wb");
        if (fp) {
            fwrite(pkt5.data(), 1, pkt5.size(), fp);
            fclose(fp);
            printf("Wrote %zu bytes to button_pressed.bin\n\n", pkt5.size());
        } else {
            perror("Failed to open button_pressed.bin");
        }
    }

    g_joysticks[0].num_axes = 2;
    g_joysticks[0].axes[0] = -1.0f; /* full left  */
    g_joysticks[0].axes[1] = 0.0f;  /* center     */
    g_joysticks[0].num_buttons = 4;
    g_joysticks[0].buttons[0] = 0; /* released    */
    g_joysticks[0].buttons[1] = 1;
    g_joysticks[0].buttons[2] = 0; /* released    */
    g_joysticks[0].buttons[3] = 1;
    g_joysticks[0].num_hats = 1;
    g_joysticks[0].hats[0] = 90;

    std::vector<uint8_t> pkt5_1 = create_robot_packet();
    printPacket("Test 5-1 (1 joystick)", pkt5_1);
    /* Write test 5 packet to binary file */
    {
        FILE *fp = fopen("button_released.bin", "wb");
        if (fp) {
            fwrite(pkt5_1.data(), 1, pkt5_1.size(), fp);
            fclose(fp);
            printf("Wrote %zu bytes to button_released.bin\n\n", pkt5_1.size());
        } else {
            perror("Failed to open button_released.bin");
        }
    }

    g_joysticks[0].num_axes = 2;
    g_joysticks[0].axes[0] = -1.0f; /* full left  */
    g_joysticks[0].axes[1] = 0.0f;  /* center     */
    g_joysticks[0].num_buttons = 4;
    g_joysticks[0].buttons[0] = 0; /* pressed    */
    g_joysticks[0].buttons[1] = 0;
    g_joysticks[0].buttons[2] = 0; /* pressed    */
    g_joysticks[0].buttons[3] = 0;
    g_joysticks[0].num_hats = 1;
    g_joysticks[0].hats[0] = 90;

    std::vector<uint8_t> pkt5_2 = create_robot_packet();
    printPacket("Test 5-2 (1 joystick)", pkt5_2);
    /* Write test 5 packet to binary file */
    {
        FILE *fp = fopen("hello.bin", "wb");
        if (fp) {
            fwrite(pkt5_2.data(), 1, pkt5_2.size(), fp);
            fclose(fp);
            printf("Wrote %zu bytes to hello.bin\n\n", pkt5_2.size());
        } else {
            perror("Failed to open hello.bin");
        }
    }

    /* ---- Test 6: with 2 joysticks ---- */
    printf("=== Test 6: Auto, enabled, Red-2, 2 joysticks, packet#7 ===\n");
    g_control_mode = DS_CONTROL_AUTONOMOUS;
    g_alliance = DS_ALLIANCE_RED;
    g_position = DS_POSITION_2;

    g_joystick_count = 2;
    /* joystick 0 same as above */
    g_joysticks[1].num_axes = 3;
    g_joysticks[1].axes[0] = 1.0f;  /* full right */
    g_joysticks[1].axes[1] = 0.5f;  /* half up    */
    g_joysticks[1].axes[2] = -0.5f; /* half down  */
    g_joysticks[1].num_buttons = 2;
    g_joysticks[1].buttons[0] = 1;
    g_joysticks[1].buttons[1] = 1;
    g_joysticks[1].num_hats = 0;

    std::vector<uint8_t> pkt6 = create_robot_packet();
    printPacket("Test 6 (2 joysticks)", pkt6);

    printf("All tests completed.\n");
    return 0;
}