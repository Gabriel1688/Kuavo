#pragma once

#include "CAN.h"
#include "Common.h"
#include "DmFrame.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <vector>

//MIT_MODE = 0x000
class Motor {
public:
    // Constructor
    Motor(MotorType motor_type, uint32_t device_id);

    // State getters
    /*
     * @brief get motor position
     * @return motor position
     */
    double getPosition() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_stateQ;
    }

    /*
     * @brief get motor Velocity
     * @return motor Velocity
     */
    double getVelocity() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_stateDq;
    }

    /*
     * @brief get motor torque
     * @return motor torque
     */
    double getTorque() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_stateTau;
    }
    int getStateTmos() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_stateTmos;
    }
    int getStateTrotor() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_stateTrotor;
    }

    // Motor property getters
    uint32_t getSendId() const { return m_deviceId; }
    uint32_t getRecvId() const { return m_deviceId + 0x10; }
    MotorType getMotorType() const { return m_motorType; }

    int getState() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_status;
    }

    MITParam getLastMitParam() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_lastMitParam;
    }

    std::chrono::steady_clock::time_point getLastUpdateTime() const {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_lastUpdateTime;
    }

    // Enable status getters
    bool isEnabled() const { return m_enabled; }

    // Parameter methods
    double getParam(int RID) const;

    // Static methods for motor properties
    static LimitParam getLimitParam(MotorType motor_type);

    void prepareWait();
    bool waitResponse();

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

    // State update methods
    void updateState(int status, double q, double dq, double tau, int tmos, int trotor);
    void setEnabled(bool enabled);
    void setTempParam(int RID, int val);
    void setStateTmos(int tmos) {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_stateTmos = tmos;
    }
    void setStateTrotor(int trotor) {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_stateTrotor = trotor;
    }

private:
    void sendMessage(dataframe_t &dataFrame, int len = 8, int command_id = 0, bool reply = false);
    StateResult parseMotorStateData(const std::vector<uint8_t> &data);
    ParamResult parseMotorParamData(const std::vector<uint8_t> &data);
    void notify();

    // Motor identifiers
    uint32_t m_deviceId;
    MotorType m_motorType;

    // Enable status
    std::atomic<bool> m_enabled{false};
    int m_status;

    // Current state
    double m_stateQ, m_stateDq, m_stateTau;
    int m_stateTmos, m_stateTrotor;
    MITParam m_lastMitParam{};
    std::chrono::steady_clock::time_point m_lastSendTime{};
    std::chrono::steady_clock::time_point m_lastUpdateTime{};

    // Motor feedback parameters  --reserved.
    // https://github.com/dmBots/motor-control-routine/blob/9137a5bdacc295ccd165ef6e7d06e649a517d096/
    // stm32%E4%BE%8B%E7%A8%8B/dm_ctrl(DM3519%20%E4%B8%80%E6%8B%96%E5%9B%9B)/User/dm_motor_drv.h#L177
    motor_ctrl_t m_motorCtrl;

    // Parameter storage
    std::map<int, double> m_paramDict;
    mutable std::mutex m_stateMutex;
    std::mutex m_requestMutex;
    std::condition_variable m_requestCv;
    bool m_completed{false};
    bool m_requestPending{false};

    std::shared_ptr<CAN> m_canHandle;
    client_observer_t<uint8_t> m_observer;
};
