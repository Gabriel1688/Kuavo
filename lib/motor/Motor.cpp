#include "Motor.h"
#include "common/Config.h"
#include "spdlog/spdlog.h"
#include <functional>
#include <stdexcept>
#include <string>
#include <utility/Utility.h>

Motor::Motor(MotorType motor_type, uint32_t device_id)
    : m_deviceId(device_id),
      m_motorType(motor_type),
      m_stateQ(0.0),
      m_stateDq(0.0),
      m_stateTau(0.0),
      m_stateTmos(0),
      m_stateTrotor(0) {
    m_canHandle = std::make_shared<CAN>(device_id);
    m_observer.wantedIP = Config::instance().motor().observerIp;
    m_observer.packetHandler = std::bind(&Motor::callback, this, std::placeholders::_1, std::placeholders::_2);
    m_canHandle->registrateCallback(m_deviceId, m_observer);
}

void Motor::setEnabled(bool enabled) { m_enabled.store(enabled); }

double Motor::getParam(int RID) const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    auto it = m_paramDict.find(RID);
    return (it != m_paramDict.end()) ? it->second : -1;
}

void Motor::setTempParam(int RID, int val) {
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_paramDict[RID] = val;
    }
    notify();
}

// State update methods
void Motor::updateState(int status, double q, double dq, double tau, int tmos, int trotor) {
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_status = status;
        m_stateQ = q;
        m_stateDq = dq;
        m_stateTau = tau;
        m_stateTmos = tmos;
        m_stateTrotor = trotor;
    }
    notify();
}

void Motor::prepareWait() {
    std::lock_guard<std::mutex> lock(m_requestMutex);
    m_completed = false;
    m_requestPending = true;
}

bool Motor::waitResponse() {
    std::unique_lock<std::mutex> lock(m_requestMutex);
    bool result = m_requestCv.wait_for(lock, std::chrono::milliseconds{200}, [this] { return m_completed; });
    m_requestPending = false;

    if (!result) {
        SPDLOG_ERROR("Motor[{}]::wait_response failed to get response within 200ms.", m_deviceId);
    }
    return result;
}

void Motor::notify() {
    {
        std::lock_guard<std::mutex> lock(m_requestMutex);
        if (!m_requestPending) return;
        m_completed = true;
    }
    m_requestCv.notify_one();
}

StateResult Motor::parseMotorStateData(const std::vector<uint8_t> &data) {
    if (data.size() < 8) {
        return {0, 0, 0, 0, 0, false};
    }

    // Parse state data
    uint8_t status = data[0] & 0x0f;//error status
    uint16_t q_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    uint16_t dq_uint = (static_cast<uint16_t>(data[3]) << 4) | (static_cast<uint16_t>(data[4]) >> 4);
    uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5];
    int t_mos = static_cast<int>(data[6]);
    int t_rotor = static_cast<int>(data[7]);

    // Convert to physical values
    LimitParam limits = MOTOR_LIMIT_PARAMS[static_cast<int>(m_motorType)];
    double recv_q = utility::uintToDouble(q_uint, -limits.pMax, limits.pMax, 16);
    double recv_dq = utility::uintToDouble(dq_uint, -limits.vMax, limits.vMax, 12);
    double recv_tau = utility::uintToDouble(tau_uint, -limits.tMax, limits.tMax, 12);

    return {status, recv_q, recv_dq, recv_tau, t_mos, t_rotor, true};
}

ParamResult Motor::parseMotorParamData(const std::vector<uint8_t> &data) {
    if (data.size() < 8)
        return {0, NAN, false};
    // Read or write Register parameter
    if ((data[2] == 0x33 || data[2] == 0x55)) {
        uint8_t RID = data[3];
        double num;
        // check data type of RID
        if ((7 <= RID && RID <= 10) || (13 <= RID && RID <= 16) || (35 <= RID && RID <= 36)) {
            num = utility::uint8sToUint32(data[4], data[5], data[6], data[7]);
        } else {
            std::array<uint8_t, 4> float_bytes = {data[4], data[5], data[6], data[7]};
            num = utility::uint8sToFloat(float_bytes);
        }
        return {RID, num, true};
    } else {
        return {0, NAN, false};
    }
}

void Motor::enableMotor() {
    dataframe_t data(dataframe_enable_motor_t{});
    sendMessage(data);
}

void Motor::disableMotor() {
    dataframe_t data(dataframe_disable_motor_t{});
    sendMessage(data);
}

void Motor::setZeroCommand() {
    dataframe_t data(dataframe_set_zero_position_t{});
    sendMessage(data);
}

void Motor::clearMotorError() {
    dataframe_t data(dataframe_clear_error_t{});
    sendMessage(data);
}

void Motor::setMitControl(const MITParam &mit_param) {
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_lastMitParam = mit_param;
    }
    uint16_t kp_uint = utility::doubleToUint(mit_param.kp, 0, 500, 12);
    uint16_t kd_uint = utility::doubleToUint(mit_param.kd, 0, 5, 12);

    // Get motor limits based on type
    LimitParam limits = MOTOR_LIMIT_PARAMS[static_cast<int>(m_motorType)];
    uint16_t q_uint = utility::doubleToUint(mit_param.q, -(double) limits.pMax, (double) limits.pMax, 16);
    uint16_t dq_uint = utility::doubleToUint(mit_param.dq, -(double) limits.vMax, (double) limits.vMax, 12);
    uint16_t tau_uint = utility::doubleToUint(mit_param.tau, -(double) limits.tMax, (double) limits.tMax, 12);

    dataframe_t df;
    df.data[0] = static_cast<uint8_t>((q_uint >> 8) & 0xFF);
    df.data[1] = static_cast<uint8_t>(q_uint & 0xFF);
    df.data[2] = static_cast<uint8_t>(dq_uint >> 4);
    df.data[3] = static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF));
    df.data[4] = static_cast<uint8_t>(kp_uint & 0xFF);
    df.data[5] = static_cast<uint8_t>(kd_uint >> 4);
    df.data[6] = static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF));
    df.data[7] = static_cast<uint8_t>(tau_uint & 0xFF);
    sendMessage(df);
}

void Motor::setPosvelControl(const PosVelParam &posvel_param) {
    auto pb = utility::floatToUint8s(static_cast<float>(posvel_param.q));
    auto vb = utility::floatToUint8s(static_cast<float>(posvel_param.dq));

    dataframe_t df;
    memcpy(df.data, &pb, sizeof(float));
    memcpy(df.data + 4, &vb, sizeof(float));
    sendMessage(df);
}

void Motor::getMotorStatus() {
    dataframe_t data;
    data.updateMotorStatus.can_id = m_deviceId;
    data.updateMotorStatus.cmd[0] = 0xcc;
    data.updateMotorStatus.cmd[1] = 0;
    prepareWait();
    sendMessage(data, 8, CMD_API_GET_MOTOR_STATUS, true);
    waitResponse();
}

void Motor::getRegParam(int RID) {
    dataframe_t data;
    data.registerParam.can_id = m_deviceId;
    data.registerParam.cmd = 0x33;
    data.registerParam.reg_id = RID;
    prepareWait();
    sendMessage(data, 8, CMD_API_GET_MOTOR_PARAMETERS, true);
    waitResponse();
}

void Motor::writeRegParam(int RID, int val) {
    dataframe_t data;
    data.registerParam.can_id = m_deviceId;
    data.registerParam.cmd = 0x55;
    data.registerParam.reg_id = RID;
    //data.writeRegisterParam.data=val;
    sendMessage(data, 8, CMD_API_WRITE_MOTOR_PARAMETERS);
}

void Motor::saveRegParam(int RID) {
    dataframe_t data;
    data.registerParam.can_id = m_deviceId;
    data.registerParam.cmd = 0xaa;
    data.registerParam.reg_id = RID;
    sendMessage(data, 8, CMD_API_SAVE_MOTOR_PARAMETERS);
}

void Motor::callback(const uint8_t *msg, size_t size) {
    std::vector<uint8_t> data(msg, msg + size);

    // Auto-detect response type from data content
    ParamResult paramResult = parseMotorParamData(data);
    if (paramResult.valid) {
        setTempParam(paramResult.rid, paramResult.value);
        return;
    }

    StateResult stateResult = parseMotorStateData(data);
    if (stateResult.valid) {
        updateState(stateResult.state, stateResult.position, stateResult.velocity,
                    stateResult.torque, stateResult.t_mos, stateResult.t_rotor);
        return;
    }

    SPDLOG_WARN("Unrecognized message can_id: {}", getRecvId());
}

void Motor::sendMessage(dataframe_t &dataFrame, int len, int command_id, bool reply) {
    m_canHandle->writePacket(dataFrame.data, 8, command_id, reply);
}

LimitParam Motor::getLimitParam(MotorType motor_type) {
    size_t index = static_cast<size_t>(motor_type);
    if (index >= MOTOR_LIMIT_PARAMS.size()) {
        throw std::invalid_argument("Invalid motor type: " + std::to_string(static_cast<int>(motor_type)));
    }
    return MOTOR_LIMIT_PARAMS[index];
}