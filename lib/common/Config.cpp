#include "Config.h"

#include <dynacore_yaml-cpp/yaml.h>

#include <stdexcept>

#include "spdlog/spdlog.h"

namespace YAML = dynacore_YAML;

// ---------- static state -----------------------------------------------------
std::string Config::s_yamlPath;

void Config::init(const std::string &yamlPath) {
    s_yamlPath = yamlPath;
}

Config &Config::instance() {
    static Config cfg;
    static bool loaded = false;
    if (!loaded) {
        std::string path = s_yamlPath.empty() ? "../config/config.yaml" : s_yamlPath;
        cfg.load(path);
        loaded = true;
    }
    return cfg;
}

// ---------- load -------------------------------------------------------------
void Config::load(const std::string &yamlPath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yamlPath);
    } catch (const std::exception &e) {
        throw std::runtime_error("Config: cannot open " + yamlPath + " : " + e.what());
    }

    // ---- MQTT ----------------------------------------------------------------
    if (root["mqtt"]) {
        auto mq = root["mqtt"];
        if (mq["mode"])
            m_mqtt.mode = mq["mode"].as<std::string>();
        if (mq["client_id"])
            m_mqtt.clientId = mq["client_id"].as<std::string>();
        if (mq["username"])
            m_mqtt.username = mq["username"].as<std::string>();
        if (mq["password"])
            m_mqtt.password = mq["password"].as<std::string>();
        if (mq["address"])
            m_mqtt.address = mq["address"].as<std::string>();
        if (mq["mqtt_broker"])
            m_mqtt.broker = mq["mqtt_broker"].as<std::string>();
        if (mq["host"])
            m_mqtt.host = mq["host"].as<std::string>();
        if (mq["port"])
            m_mqtt.port = mq["port"].as<std::string>();
        if (mq["mqtt_port"])
            m_mqtt.mqttPort = mq["mqtt_port"].as<std::string>();
        if (mq["in_port"])
            m_mqtt.inPort = mq["in_port"].as<int>();
        if (mq["out_port"])
            m_mqtt.outPort = mq["out_port"].as<int>();
        if (mq["qos"])
            m_mqtt.qos = mq["qos"].as<int>();
        if (mq["robot_id"])
            m_mqtt.robotId = mq["robot_id"].as<int>();
        if (mq["topics"]) {
            for (auto it = mq["topics"].begin(); it != mq["topics"].end(); ++it) {
                m_mqtt.topics.push_back(it->as<std::string>());
            }
        }
    }

    // ---- UDP -----------------------------------------------------------------
    if (root["udp"]) {
        auto ud = root["udp"];
        if (ud["server_ip"])
            m_udp.serverIp = ud["server_ip"].as<std::string>();
        if (ud["base_local_port"])
            m_udp.baseLocalPort = ud["base_local_port"].as<int>();
        if (ud["base_remote_port"])
            m_udp.baseRemotePort = ud["base_remote_port"].as<int>();
        if (ud["client_ip"]) {
            auto ci = ud["client_ip"];
            if (ci["left"])
                m_udp.clientIpLeft = ci["left"].as<std::string>();
            if (ci["right"])
                m_udp.clientIpRight = ci["right"].as<std::string>();
        }
    }
    // ---- IMU -----------------------------------------------------------------
    if (root["imu"]) {
        auto imu = root["imu"];
        if (imu["server_ip"])
            m_imu.serverIp = imu["server_ip"].as<std::string>();
        if (imu["local_port"])
            m_imu.localPort = imu["local_port"].as<int>();
        if (imu["value_mode"]) {
            m_imu.type = (imu["value_mode"].as<std::string>() == "float" ? ImuType::FLOAT : ImuType::INT);
        }
        if (imu["base_id"])
            m_imu.baseId = imu["base_id"].as<int>();
    }
    // ---- Motor ---------------------------------------------------------------
    if (root["motor"]) {
        auto mo = root["motor"];
        if (mo["observer_ip"])
            m_motor.observerIp = mo["observer_ip"].as<std::string>();
        if (mo["max_can_device"])
            m_motor.maxCanDevice = mo["max_can_device"].as<int>();
        if (mo["motors_per_leg"])
            m_motor.motorsPerLeg = mo["motors_per_leg"].as<int>();

        // motor types
        if (mo["types"]) {
            for (auto it = mo["types"].begin(); it != mo["types"].end(); ++it) {
                MotorType mt;
                mt.name = it->first.as<std::string>();
                auto child = it->second;
                if (child["id"])
                    mt.id = child["id"].as<int>();
                if (child["position_max"])
                    mt.positionMax = child["position_max"].as<double>();
                if (child["velocity_max"])
                    mt.velocityMax = child["velocity_max"].as<double>();
                if (child["torque_max"])
                    mt.torqueMax = child["torque_max"].as<double>();
                m_motor.types.push_back(mt);
            }
        }

        // legs
        if (mo["legs"]) {
            for (auto it = mo["legs"].begin(); it != mo["legs"].end(); ++it) {
                Leg leg;
                leg.name = it->first.as<std::string>();
                auto child = it->second;
                if (child["base_id"])
                    leg.baseId = child["base_id"].as<int>();
                if (child["server_id"])
                    leg.serverId = child["server_id"].as<int>();
                if (child["motor_type"])
                    leg.motorType = child["motor_type"].as<std::string>();
                if (child["device_ids"]) {
                    for (auto jt = child["device_ids"].begin(); jt != child["device_ids"].end(); ++jt) {
                        leg.deviceIds.push_back(jt->as<int>());
                    }
                }
                m_motor.legs.push_back(leg);
            }
        }
        // ---- Logger -----------------------------------------------------------------
        if (root["logger"]) {
            auto logger = root["logger"];
            if (logger["path"])
                m_logger.path = logger["path"].as<std::string>();
            if (logger["level"])
                m_logger.level = logger["level"].as<std::string>();
            if (logger["maxSize"]) {
                m_logger.maxSize = logger["maxSize"].as<int>();
            }
            if (logger["rotation"])
                m_logger.rotation = logger["rotation"].as<int>();
        }
    }

    // ---- Data Logger --------------------------------------------------------
    if (root["data_logger"]) {
        auto dl = root["data_logger"];
        if (dl["enabled"])
            m_dataLogger.enabled = dl["enabled"].as<bool>();
        if (dl["downsample_every"])
            m_dataLogger.downsampleEvery = dl["downsample_every"].as<int>();
        if (dl["ring_buffer_capacity"])
            m_dataLogger.ringBufferCapacity = dl["ring_buffer_capacity"].as<int>();
    }

    // ---- Driver Station ---------------------------------------------------------
    if (root["driver_station"]) {
        auto ds = root["driver_station"];
        if (ds["udp_port"])
            m_driverStation.udpPort = ds["udp_port"].as<int>();
        if (ds["buttons"]) {
            auto buttons = ds["buttons"];
            for (auto it = buttons.begin(); it != buttons.end(); ++it) {
                m_driverStation.buttons[it->first.as<std::string>()] = it->second.as<std::string>();
            }
        }
    }

    SPDLOG_INFO("Config loaded from {}", yamlPath);
}

// ---------- finders ----------------------------------------------------------
const Config::MotorType *Config::findMotorType(const std::string &name) const {
    for (auto &mt : m_motor.types) {
        if (mt.name == name)
            return &mt;
    }
    return nullptr;
}

const Config::Leg *Config::findLeg(const std::string &name) const {
    for (auto &l : m_motor.legs) {
        if (l.name == name)
            return &l;
    }
    return nullptr;
}
