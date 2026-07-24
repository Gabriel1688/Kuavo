#pragma once

#include "CAN.h"
#include "Common.h"
#include "DmFrame.h"
#include <atomic>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace mercury { class MotorParamCache; }

static_assert(std::atomic<double>::is_always_lock_free,
              "std::atomic<double> must be lock-free for real-time use");

//MIT_MODE = 0x000
class Motor {
public:
    // Constructor
    Motor(MotorType motor_type, uint32_t device_id, mercury::MotorParamCache* param_cache = nullptr);

    // State getters
    /*
     * @brief get motor position
     * @return motor position
     */
    double getPosition() const {
        return m_stateQ.load(std::memory_order_acquire);
    }

    /*
     * @brief get motor Velocity
     * @return motor Velocity
     */
    double getVelocity() const {
        return m_stateDq.load(std::memory_order_acquire);
    }

    /*
     * @brief get motor torque
     * @return motor torque
     */
    double getTorque() const {
        return m_stateTau.load(std::memory_order_acquire);
    }
    int getStateTmos() const {
        return m_stateTmos.load(std::memory_order_acquire);
    }
    int getStateTrotor() const {
        return m_stateTrotor.load(std::memory_order_acquire);
    }

    // Motor property getters
    uint32_t getSendId() const { return m_deviceId; }
    uint32_t getRecvId() const { return m_deviceId + 0x10; }
    MotorType getMotorType() const { return m_motorType; }

    int getState() const {
        return m_status.load(std::memory_order_acquire);
    }

    MITParam getLastMitParam() const {
        std::lock_guard<std::mutex> lock(m_commandMutex);
        return m_lastMitParam;
    }

    uint64_t getLastUpdateTime() const {
        return m_lastUpdateTime.load(std::memory_order_acquire);
    }

    // Enable status getters
    bool isEnabled() const { return m_enabled.load(); }

    // Parameter methods
    double getParam(int RID) const;

    // Static methods for motor properties
    static LimitParam getLimitParam(MotorType motor_type);

    // CAN Commands
    void enableMotor();
    void disableMotor();
    void setZeroCommand();
    void clearMotorError();
    void getMotorStatus();
    void getRegParam(int RID);
    void writeRegParam(int RID, int val);
    void saveRegParam(int RID);

    //Only support the MIT control mode.
    void setMitControl(const MITParam &mit_param);
    void setPosvelControl(const PosVelParam &posvel_param);

    void callback(const uint8_t *msg, size_t size);

    // Static trampoline for function-pointer-based callback dispatch.
    // Avoids std::function/std::bind overhead in the hot packet path.
    static void packetTrampoline(void* ctx, const uint8_t* data, size_t len) {
        static_cast<Motor*>(ctx)->callback(data, len);
    }

    // State update methods
    void updateState(int status, double q, double dq, double tau, int tmos, int trotor);
    void setEnabled(bool enabled);
    void setTempParam(int RID, int val);
    void setStateTmos(int tmos) {
        m_stateTmos.store(tmos, std::memory_order_release);
    }
    void setStateTrotor(int trotor) {
        m_stateTrotor.store(trotor, std::memory_order_release);
    }

private:
    void sendMessage(dataframe_t &dataFrame, int len = 8, int command_id = 0, bool reply = false);
    StateResult parseMotorStateData(const std::vector<uint8_t> &data);
    ParamResult parseMotorParamData(const std::vector<uint8_t> &data);

    // Motor identifiers
    uint32_t m_deviceId;
    MotorType m_motorType;

    // Enable status
    std::atomic<bool> m_enabled{false};
    std::atomic<int> m_status;

    // Current state
    std::atomic<double> m_stateQ, m_stateDq, m_stateTau;
    std::atomic<int> m_stateTmos, m_stateTrotor;
    MITParam m_lastMitParam{};
    uint64_t m_lastSendTimeNs{};  // CLOCK_MONOTONIC nanoseconds
    std::atomic<uint64_t> m_lastUpdateTime{};

    // Motor feedback parameters  --reserved.
    // https://github.com/dmBots/motor-control-routine/blob/9137a5bdacc295ccd165ef6e7d06e649a517d096/
    // stm32%E4%BE%8B%E7%A8%8B/dm_ctrl(DM3519%20%E4%B8%80%E6%8B%96%E5%9B%9B)/User/dm_motor_drv.h#L177
    motor_ctrl_t m_motorCtrl;

    // Parameter storage
    std::map<int, double> m_paramDict;
    mutable std::mutex m_paramMutex;
    mutable std::mutex m_commandMutex;

    std::mutex m_transactionMutex;  // serializes commands per motor

    mercury::MotorParamCache* m_paramCache = nullptr;

    std::shared_ptr<CAN> m_canHandle;
    client_observer_t<uint8_t> m_observer;
};
