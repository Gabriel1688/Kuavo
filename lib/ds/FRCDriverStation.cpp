#include "DriverStation.h"
#include "DriverStationTypes.h"
#include "EventVector.h"
#include "mqtt/MqttClient.h"
#include <atomic>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>

static const int JOYSTICK_POS = 19;
static_assert(sizeof(int32_t) >= sizeof(int), "FRC_NetworkComm status variable is larger than 32 bits");

namespace {
struct HAL_JoystickAxesInt {
    int16_t count;
    int16_t axes[HAL_kMaxJoystickAxes];
};
}// namespace

struct JoystickDataCache {
    JoystickDataCache() { std::memset(this, 0, sizeof(*this)); }

    void Update(const char *payload);

    HAL_JoystickAxes axes[HAL_kMaxJoysticks];
    HAL_JoystickPOVs povs[HAL_kMaxJoysticks];
    HAL_JoystickButtons buttons[HAL_kMaxJoysticks];
    HAL_ControlWord controlWord;
};

static_assert(std::is_standard_layout_v<JoystickDataCache>);

struct FRCDriverStation {
    EventVector newDataEvents;
};

static ::FRCDriverStation *driverStation;

// Message and Data variables
static std::mutex msgMutex;

static int32_t
HAL_GetJoystickAxesInternal(__attribute__((unused)) const char *payload, int32_t joystickNum, HAL_JoystickAxes *axes) {
    HAL_JoystickAxesInt netcommAxes;
    int32_t joystick_size = payload[JOYSTICK_POS];

    if (joystick_size <= joystickNum)//Invalid joystickNum  --how to know joystick is plugout?
    {
        std::cout << "HAL_GetJoystickAxesInternal PARAMETER_OUT_OF_RANGE joystickNum : " << joystickNum << std::endl;
        return 0;
    }

    int offset = JOYSTICK_POS + 2;
    netcommAxes.count = payload[offset];// NumAxes
    offset++;

    for (int i = 0; i < netcommAxes.count; i++) {
        netcommAxes.axes[i] = payload[offset];
        offset++;
    }

    // copy integer values to double values
    axes->count = netcommAxes.count;
    // current scaling is -128 to 127, can easily be patched in the future by changing this function.
    for (int32_t i = 0; i < netcommAxes.count; i++) {
        int8_t value = netcommAxes.axes[i];
        axes->raw[i] = value;
        if (value < 0) {
            axes->axes[i] = value / 128.0;
        } else {
            axes->axes[i] = value / 127.0;
        }
    }
    return 0;
}

static int32_t
HAL_GetJoystickPOVsInternal(__attribute__((unused)) const char *payload, int32_t joystickNum, HAL_JoystickPOVs *povs) {
    int32_t joystick_size = payload[JOYSTICK_POS];
    if (joystick_size <= joystickNum)//Invalid joystickNum  --how to know joystick is plugout?
    {
        std::cout << "HAL_GetJoystickAxesInternal PARAMETER_OUT_OF_RANGE joystickNum : " << joystickNum << std::endl;
        return 0;
    }

    int offset = 6 + JOYSTICK_POS + payload[JOYSTICK_POS + 2];// NumHats
    povs->count = payload[offset];
    offset++;
    for (int i = 0; i < povs->count; i++) {
        povs->povs[i] = (payload[offset] << 8) + payload[offset + 1];
        offset = offset + 2;
    }
    return 0;
}

static int32_t HAL_GetJoystickButtonsInternal(__attribute__((unused)) const char *payload, int32_t joystickNum,
                                              HAL_JoystickButtons *buttons) {
    int32_t joystick_size = payload[JOYSTICK_POS];
    if (joystick_size <= joystickNum)//Invalid joystickNum  --how to know joystick is plugout?
    {
        std::cout << "HAL_GetJoystickAxesInternal PARAMETER_OUT_OF_RANGE joystickNum : " << joystickNum << std::endl;
        return 0;
    }

    int offset = JOYSTICK_POS + 3 + payload[JOYSTICK_POS + 2];// NumAxes
    buttons->count = payload[offset];

    buttons->buttons = (payload[offset + 1] << 8) + payload[offset + 2];
    return 0;
}

void JoystickDataCache::Update(const char *payload) {
    HAL_GetJoystickAxesInternal(payload, 0, &axes[0]);
    HAL_GetJoystickPOVsInternal(payload, 0, &povs[0]);
    HAL_GetJoystickButtonsInternal(payload, 0, &buttons[0]);
    memcpy((unsigned char *) &controlWord, (unsigned char *) &payload[3], 1);
}

#define CHECK_JOYSTICK_NUMBER(stickNum)                    \
    if ((stickNum) < 0 || (stickNum) >= HAL_kMaxJoysticks) \
    return -1

static HAL_ControlWord newestControlWord;
static JoystickDataCache caches[3];
static JoystickDataCache *currentRead = &caches[0];
static JoystickDataCache *currentReadLocal = &caches[0];
static std::atomic<JoystickDataCache *> currentCache{nullptr};
static JoystickDataCache *lastGiven = &caches[1];
static JoystickDataCache *cacheToUpdate = &caches[2];

static std::mutex cacheMutex;

extern "C" {

void InitializeFRCDriverStation() {
    std::memset(&newestControlWord, 0, sizeof(newestControlWord));
    static FRCDriverStation ds;
    driverStation = &ds;
}

int32_t HAL_GetControlWord(HAL_ControlWord *controlWord) {
    std::scoped_lock lock{cacheMutex};
    *controlWord = newestControlWord;
    return 0;
}

int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes *axes) {
    CHECK_JOYSTICK_NUMBER(joystickNum);
    std::scoped_lock lock{cacheMutex};
    *axes = currentRead->axes[joystickNum];
    return 0;
}

int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs *povs) {
    CHECK_JOYSTICK_NUMBER(joystickNum);
    std::scoped_lock lock{cacheMutex};
    *povs = currentRead->povs[joystickNum];
    return 0;
}

int32_t HAL_GetJoystickButtons(int32_t joystickNum, HAL_JoystickButtons *buttons) {
    CHECK_JOYSTICK_NUMBER(joystickNum);
    std::scoped_lock lock{cacheMutex};
    *buttons = currentRead->buttons[joystickNum];
    return 0;
}

void HAL_GetAllJoystickData(HAL_JoystickAxes *axes, HAL_JoystickPOVs *povs, HAL_JoystickButtons *buttons) {
    std::scoped_lock lock{cacheMutex};
    std::memcpy(axes, currentRead->axes, sizeof(currentRead->axes));
    std::memcpy(povs, currentRead->povs, sizeof(currentRead->povs));
    std::memcpy(buttons, currentRead->buttons, sizeof(currentRead->buttons));
}

static void newDataOccur(const void *payload, __attribute__((unused)) uint32_t payload_len) {
    cacheToUpdate->Update(static_cast<const char *>(payload));

    JoystickDataCache *given = cacheToUpdate;
    JoystickDataCache *prev = currentCache.exchange(cacheToUpdate);
    if (prev == nullptr) {
        cacheToUpdate = currentReadLocal;
        currentReadLocal = lastGiven;
    } else {
        // Current read local does not update
        cacheToUpdate = prev;
    }
    lastGiven = given;

    if (driverStation)
        driverStation->newDataEvents.wakeup();
}

HAL_Bool HAL_RefreshDSData(void) {
    HAL_ControlWord controlWord;
    std::memset(&controlWord, 0, sizeof(controlWord));

    controlWord = lastGiven->controlWord;

    // FRC_NetworkCommunication_getControlWord(reinterpret_cast<ControlWord_t *>(&controlWord));
    JoystickDataCache *prev;
    {
        std::scoped_lock lock{cacheMutex};
        prev = currentCache.exchange(nullptr);
        if (prev != nullptr) {
            currentRead = prev;
        }
        // If newest state shows we have a DS attached, just use the
        // control word out of the cache, As it will be the one in sync
        // with the data. If no data has been updated, at this point,
        // and a DS wasn't attached previously, this will still return
        // a zeroed out control word, with is the correct state for
        // no new data.

        if (!controlWord.dsAttached) {
            // If the DS is not attached, we need to zero out the control word.
            // This is because HAL_RefreshDSData is called asynchronously from
            // the DS data. The dsAttached variable comes directly from netcomm
            // and could be updated before the caches are. If that happens,
            // we would end up returning the previous cached control word,
            // which is out of sync with the current control word and could
            // break invariants such as which alliance station is in used.
            // Also, when the DS has never been connected the rest of the fields
            // in control word are garbage, so we also need to zero out in that
            // case too
            std::memset(&currentRead->controlWord, 0, sizeof(currentRead->controlWord));
        }
        newestControlWord = currentRead->controlWord;
    }
    return prev != nullptr;
}

void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle) {
    if (driverStation)
        driverStation->newDataEvents.add(handle);
}

void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle) {
    if (driverStation)
        driverStation->newDataEvents.remove(handle);
}

namespace hal {
void InitializeDriverStation() {
    // Ensure driverStation is initialized before installing the callback,
    // so newDataOccur never dereferences a null pointer.
    InitializeFRCDriverStation();
    // Set up the occur function internally with NetComm
    g_mqttClient_ptr.load()->setOccurFuncPointer(newDataOccur);
    //https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/hal/src/main/native/athena/HAL.cpp#L568
    //https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/hal/src/main/native/athena/FRCDriverStation.cpp#L587
}
}// namespace hal
}