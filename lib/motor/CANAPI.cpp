#include "CANAPI.h"
#include "UdpServer.h"
#include "common/Config.h"
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

#define HAL_HANDLE_ERROR -1098
#define HAL_CAN_SEND_PERIOD_STOP_REPEATING -1
#define HAL_CAN_SEND_PERIOD_NO_REPEAT 0
#define HAL_CAN_IS_FRAME_REMOTE 0x80000000
#define HAL_CAN_TIMEOUT -1154
//https://github.com/wpilibsuite/frc-docs/blob/main/source/docs/software/can-devices/can-addressing.rst
//https://github.com/wpilibsuite/frc-docs/blob/main/source/docs/software/can-devices/third-party-devices.rst
//https://github.com/REVrobotics/node-can-bridge/blob/main/src/canWrapper.cc
//https://github.com/REVrobotics/node-can-bridge/blob/main/test/test_binding.js
//https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/e0ae2fd0c2a55d9719087505cbec8be9fc4293bf/src/main/cpp/subsystems/MAXSwerveModule.cpp#L15
//https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/e0ae2fd0c2a55d9719087505cbec8be9fc4293bf/src/main/include/Configs.h#L10

/*
     : m_drivingSpark(drivingCANId, SparkMax::MotorType::kBrushless),
      m_turningSpark(turningCANId, SparkMax::MotorType::kBrushless) {
  // Apply the respective configurations to the SPARKS. Reset parameters before
  // applying the configuration to bring the SPARK to a known good state.
  // Persist the settings to the SPARK to avoid losing them on a power cycle.
  m_drivingSpark.Configure(Configs::MAXSwerveModule::DrivingConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
  m_turningSpark.Configure(Configs::MAXSwerveModule::TurningConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);


class MAXSwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig() {
    static SparkMaxConfig drivingConfig{};

    // Use module constants to calculate conversion factors and feed forward
    // gain.
    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;
    double drivingVelocityFeedForward =
        1 / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    drivingConfig.encoder
        .PositionConversionFactor(drivingFactor)          // meters
        .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    drivingConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(0.04, 0, 0)
        .VelocityFF(drivingVelocityFeedForward)
        .OutputRange(-1, 1);

    return drivingConfig;
  }
//https://github.com/frc3512/Robot-2022/blob/01407465389466b36ccae6731e987436331c4ef7/src/main/include/HWConfig.hpp#L79
 * */

static std::map<HAL_CANHandle, std::shared_ptr<CANStorage>> *canHandles;
std::mutex canHandlesMutex;

//UdpServer *g_server;
//static UdpServer *g_server;
//static UnlimitedHandleResource<HAL_CANHandle, CANStorage, HAL_HandleEnum::CAN>*  canHandles;

namespace hal {
namespace init {
void InitializeCANAPI(int instanceId) {
    static std::map<HAL_CANHandle, std::shared_ptr<CANStorage>> cH;
    canHandles = &cH;
    UdpServer::getInstance(instanceId).setCanHandles(canHandles);
    UdpServer::getInstance(instanceId).start();
}
}// namespace init
}// namespace hal

static CANFrameId CreateCANId(CANStorage *storage, int32_t apiId, HAL_CANHandle handle) {
    CANFrameId createdId;
    switch (storage->manufacturer) {
    case HAL_CAN_Man_Dm://DmBot can FrameID
        // API type which has fixed CANID (0x7FF-broadcast CAN ID)
        if ((apiId == 0x01) || (apiId == 0x02) || (apiId == 0x03) || (apiId == 0x04)) {
            createdId.forwardCANId = 0x7FF;
        } else {
            createdId.forwardCANId = storage->deviceId & 0x3F;
        }
        createdId.deviceId = storage->deviceId;
        createdId.replyCANId = storage->masterId;
        createdId.hanlde = handle;
        break;
    default:
        break;
    }
    return createdId;
}

extern "C" {
HAL_CANHandle HAL_InitializeCAN(HAL_CANManufacturer manufacturer,
                                int32_t deviceId, HAL_CANDeviceType deviceType,
                                __attribute__((unused)) int32_t *status) {
    //hal::init::CheckInit();
    auto can = std::make_shared<CANStorage>();
    int handle = canHandles->size();
    canHandles->insert(std::make_pair(handle, can));

    can->deviceId = deviceId;
    can->masterId = deviceId + 0x10;
    can->deviceType = deviceType;
    can->manufacturer = manufacturer;
    return handle;
}

void HAL_CleanCAN(HAL_CANHandle __attribute__((unused)) handle) {
#if 0
    auto data = canHandles->Free(handle);
    if (data == nullptr) {
        return;
    }

    std::scoped_lock lock(data->periodicSendsMutex);

    for (auto&& i : data->periodicSends) {
        int32_t s = 0;
        auto id = CreateCANId(data.get(), i.first);
        HAL_CAN_SendMessage(id, nullptr, 0, HAL_CAN_SEND_PERIOD_STOP_REPEATING, &s);
        i.second = -1;
    }
#endif
}

void HAL_WriteCANPacket(HAL_CANHandle handle, const uint8_t *data, int32_t length, int32_t apiId, int32_t *status, bool reply) {
    auto it = canHandles->find(handle);
    if (it == canHandles->end()) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    auto can = it->second;
    auto id = CreateCANId(can.get(), apiId, handle);
    std::scoped_lock lock(can->periodicSendsMutex);
    can->periodicSends[apiId] = -1;
    if (can->deviceId < Config::instance().motor().maxCanDevice) {
        UdpServer::getInstance(0).sendMsg(id, data, length, reply, status);
    } else {
        UdpServer::getInstance(1).sendMsg(id, data, length, reply, status);
    }
    //TODO:: only wait for which reply. wpi::waitForObject(can->replyEvent.getHandle());
}

void HAL_WriteCANPacketRepeating(HAL_CANHandle handle, const uint8_t *data,
                                 int32_t length, int32_t apiId,
                                 int32_t repeatMs, int32_t *status, bool reply) {
    auto it = canHandles->find(handle);
    if (it == canHandles->end()) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    auto can = it->second;
    auto id = CreateCANId(can.get(), apiId, handle);

    std::scoped_lock lock(can->periodicSendsMutex);
    if (can->deviceId < Config::instance().motor().maxCanDevice) {
        UdpServer::getInstance(0).sendMsg(id, data, length, reply, status);
    } else {
        UdpServer::getInstance(1).sendMsg(id, data, length, reply, status);
    }
    can->periodicSends[apiId] = repeatMs;
}

void HAL_StopCANPacketRepeating(HAL_CANHandle handle, int32_t apiId, int32_t *status, bool reply) {
    auto it = canHandles->find(handle);
    if (it == canHandles->end()) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    auto can = it->second;

    auto id = CreateCANId(can.get(), apiId, handle);

    std::scoped_lock lock(can->periodicSendsMutex);
    if (can->deviceId < Config::instance().motor().maxCanDevice) {
        UdpServer::getInstance(0).sendMsg(id, nullptr, HAL_CAN_SEND_PERIOD_STOP_REPEATING, reply, status);
    } else {
        UdpServer::getInstance(1).sendMsg(id, nullptr, HAL_CAN_SEND_PERIOD_STOP_REPEATING, reply, status);
    }

    can->periodicSends[apiId] = -1;
}

void HAL_ReadCANPacketNew(HAL_CANHandle handle, int32_t apiId, uint8_t *data,
                          int32_t *length, uint64_t *receivedTimestamp,
                          int32_t *status, bool reply) {
    auto it = canHandles->find(handle);
    if (it == canHandles->end()) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    auto can = it->second;

    auto messageId = CreateCANId(can.get(), apiId, handle);
    uint8_t dataSize = 0;
    uint32_t ts = 0;
    //Note:: need to work in the async mode rather than sync mode.
    //How to store the incoming data to receives?
    if (can->deviceId < Config::instance().motor().maxCanDevice) {
        UdpServer::getInstance(0).sendMsg(messageId, nullptr, HAL_CAN_SEND_PERIOD_STOP_REPEATING, reply, status);
    } else {
        UdpServer::getInstance(1).sendMsg(messageId, nullptr, HAL_CAN_SEND_PERIOD_STOP_REPEATING, reply, status);
    }
    wpi::waitForObject(can->replyEvent.getHandle());

    if (*status == 0) {
        std::scoped_lock lock(can->receivesMutex);
        auto &msg = can->receives[messageId.replyCANId];
        msg.length = dataSize;
        msg.lastTimeStamp = ts;
        memcpy(msg.data, data, dataSize);
    }
    *length = dataSize;
    *receivedTimestamp = ts;
}
void HAL_ReadCANPacketLatest(HAL_CANHandle handle, int32_t apiId, uint8_t *data,
                             int32_t *length, uint64_t *receivedTimestamp,
                             int32_t *status) {
    auto can = canHandles->find(handle)->second;
    if (!can) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    auto messageId = CreateCANId(can.get(), apiId, handle);
    uint8_t dataSize = 0;
    uint32_t ts = 0;
    //HAL_CAN_ReceiveMessage(&messageId, 0x1FFFFFFF, data, &dataSize, &ts, status);

    wpi::waitForObject(can->replyEvent.getHandle());

    std::scoped_lock lock(can->receivesMutex);
    if (*status == 0) {
        // fresh update
        auto &msg = can->receives[messageId.replyCANId];
        msg.length = dataSize;
        *length = dataSize;
        msg.lastTimeStamp = ts;
        *receivedTimestamp = ts;
        // The NetComm call placed in data, copy into the msg
        std::memcpy(msg.data, data, dataSize);
    } else {
        auto i = can->receives.find(messageId.replyCANId);
        if (i != can->receives.end()) {
            // Read the data from the stored message into the output
            std::memcpy(data, i->second.data, i->second.length);
            *length = i->second.length;
            *receivedTimestamp = i->second.lastTimeStamp;
            *status = 0;
        }
    }
}
void HAL_ReadCANPacketTimeout(HAL_CANHandle handle, int32_t apiId,
                              uint8_t *data, int32_t *length,
                              uint64_t *receivedTimestamp, int32_t timeoutMs,
                              int32_t *status) {
    auto can = canHandles->find(handle)->second;
    if (!can) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    auto messageId = CreateCANId(can.get(), apiId, handle);
    uint8_t dataSize = 0;
    uint32_t ts = 0;
    // HAL_CAN_ReceiveMessage(&messageId, 0x1FFFFFFF, data, &dataSize, &ts, status);
    wpi::waitForObject(can->replyEvent.getHandle());
    std::scoped_lock lock(can->receivesMutex);
    if (*status == 0) {
        // fresh update
        auto &msg = can->receives[messageId.replyCANId];
        msg.length = dataSize;
        *length = dataSize;
        msg.lastTimeStamp = ts;
        *receivedTimestamp = ts;
        // The NetComm call placed in data, copy into the msg
        std::memcpy(msg.data, data, dataSize);
    } else {
        auto i = can->receives.find(messageId.replyCANId);
        if (i != can->receives.end()) {
            // Found, check if new enough

            // Get the current time point and Convert to milliseconds since epoch
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
            // Get the millisecond count
            uint32_t now = now_ms.time_since_epoch().count();

            if (now - i->second.lastTimeStamp > static_cast<uint32_t>(timeoutMs)) {
                // Timeout, return bad status
                *status = HAL_CAN_TIMEOUT;
                return;
            }
            // Read the data from the stored message into the output
            std::memcpy(data, i->second.data, i->second.length);
            *length = i->second.length;
            *receivedTimestamp = i->second.lastTimeStamp;
            *status = 0;
        }
    }
}
}// extern "C"
