#include "telemetry/DataLog.h"
#include "ds/DriverStation.h"
#include "spdlog/spdlog.h"

using json = nlohmann::json;

// ── construction ────────────────────────────────────────────────────────────

DataLog::DataLog(MqttClient &mqtt) : m_mqtt(mqtt) {}

// ── helpers ─────────────────────────────────────────────────────────────────

int64_t DataLog::now() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void DataLog::publish(const std::string &topic, const std::string &payload) {
    if (!m_mqtt.isConnected()) {
        return;
    }
    std::string t = topic;              // publish() takes non-const ref
    m_mqtt.publish(t, payload);
}

// ── driver station ──────────────────────────────────────────────────────────

void DataLog::logDriverStation() {
    HAL_ControlWord cw;
    HAL_GetControlWord(&cw);

    HAL_JoystickAxes axes;
    HAL_JoystickPOVs povs;
    HAL_JoystickButtons buttons;
    HAL_GetJoystickAxes(0, &axes);
    HAL_GetJoystickPOVs(0, &povs);
    HAL_GetJoystickButtons(0, &buttons);

    json doc;
    doc["bn"] = "driverstation";
    doc["bt"] = now();

    json entries = json::array();

    // Control word packed as a single uint32
    uint32_t cwRaw = 0;
    std::memcpy(&cwRaw, &cw, sizeof(cwRaw));
    entries.push_back({{"n", "DSControlWord"}, {"u", "bit"}, {"v", cwRaw}});

    // Joystick axes (only populated ones)
    for (int i = 0; i < axes.count && i < HAL_kMaxJoystickAxes; ++i) {
        entries.push_back({{"n", "axis" + std::to_string(i)}, {"u", "ratio"}, {"v", axes.axes[i]}});
    }

    // Joystick POVs
    for (int i = 0; i < povs.count && i < HAL_kMaxJoystickPOVs; ++i) {
        entries.push_back({{"n", "pov" + std::to_string(i)}, {"u", "deg"}, {"v", povs.povs[i]}});
    }

    // Joystick buttons (bitmask + count)
    entries.push_back({{"n", "buttons"}, {"u", "bit"}, {"v", buttons.buttons}});
    entries.push_back({{"n", "buttonCount"}, {"u", "count"}, {"v", buttons.count}});

    doc["e"] = entries;
    publish("/telemetry/driverstation", doc.dump());
}

// ── controller (MITParam per joint) ─────────────────────────────────────────

void DataLog::logController(const std::string &name,
                            const std::vector<MITParam> &params) {
    json doc;
    doc["bn"] = "controller-" + name;
    doc["bt"] = now();

    json entries = json::array();
    for (size_t j = 0; j < params.size(); ++j) {
        const auto &p = params[j];
        std::string prefix = "joint" + std::to_string(j) + "/";
        entries.push_back({{"n", prefix + "kp"},  {"u", "Nm/rad"},   {"v", p.kp}});
        entries.push_back({{"n", prefix + "kd"},  {"u", "Nm*s/rad"}, {"v", p.kd}});
        entries.push_back({{"n", prefix + "q"},   {"u", "rad"},      {"v", p.q}});
        entries.push_back({{"n", prefix + "dq"},  {"u", "rad/s"},    {"v", p.dq}});
        entries.push_back({{"n", prefix + "tau"}, {"u", "Nm"},       {"v", p.tau}});
    }

    doc["e"] = entries;
    publish("/telemetry/subsystem/" + name + "/controller", doc.dump());
}

// ── motor state feedback ────────────────────────────────────────────────────

void DataLog::logMotors(const std::string &name,
                        const std::vector<std::shared_ptr<Motor>> &motors) {
    json doc;
    doc["bn"] = "motor-" + name;
    doc["bt"] = now();

    json entries = json::array();
    for (const auto &motor : motors) {
        std::string prefix = "m" + std::to_string(motor->getSendId()) + "/";
        entries.push_back({{"n", prefix + "state"},    {"u", "enum"},  {"v", motor->getState()}});
        entries.push_back({{"n", prefix + "position"}, {"u", "rad"},   {"v", motor->getPosition()}});
        entries.push_back({{"n", prefix + "velocity"}, {"u", "rad/s"}, {"v", motor->getVelocity()}});
        entries.push_back({{"n", prefix + "torque"},   {"u", "Nm"},    {"v", motor->getTorque()}});
        entries.push_back({{"n", prefix + "t_mos"},    {"u", "Cel"},   {"v", motor->getStateTmos()}});
        entries.push_back({{"n", prefix + "t_rotor"},  {"u", "Cel"},   {"v", motor->getStateTrotor()}});
    }

    doc["e"] = entries;
    publish("/telemetry/subsystem/" + name + "/motor", doc.dump());
}

// ── IMU ─────────────────────────────────────────────────────────────────────

void DataLog::logImu(const Eigen::Vector<double, 7> &state) {
    json doc;
    doc["bn"] = "imu";
    doc["bt"] = now();

    static const char *labels[] = {
        "x", "y", "z", "qw", "qx", "qy", "qz"};

    json entries = json::array();
    for (int i = 0; i < 7; ++i) {
        entries.push_back({{"n", labels[i]},
                           {"u", i < 3 ? "m" : "1"},
                           {"v", state(i)}});
    }

    doc["e"] = entries;
    publish("/telemetry/subsystem/imu", doc.dump());
}
