#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

/**
 * Singleton that loads config/config.yaml and exposes every value that was
 * previously hard-coded across the project.
 *
 * Call Config::instance() to obtain the (lazily-initialised) singleton.
 * On the first call it reads config.yaml from the path given by init(),
 * or falls back to "../config/config.yaml".
 */
class Config {
public:
    // --- MQTT -----------------------------------------------------------------
    struct Mqtt {
        std::string mode;
        std::string clientId;
        std::string username;
        std::string password;
        std::string address;
        std::string broker;
        std::string host;
        std::string port;
        std::string mqttPort;
        int inPort = 0;
        int outPort = 0;
        int qos = 0;
        int robotId = 1;
        std::vector<std::string> topics;
    };

    // --- UDP ------------------------------------------------------------------
    struct Udp {
        std::string serverIp;
        int baseLocalPort = 0;
        int baseRemotePort = 0;
        std::string clientIpLeft;
        std::string clientIpRight;
    };

    enum class ImuType { INT,
                         FLOAT };
    // --- UDP ------------------------------------------------------------------
    struct Imu {
        std::string serverIp;
        int localPort = 0;
        ImuType type = ImuType::FLOAT;
        int baseId = 0x514;
    };

    // --- Motor / Leg ----------------------------------------------------------
    struct MotorType {
        std::string name;
        int id = 0;
        double positionMax = 0;
        double velocityMax = 0;
        double torqueMax = 0;
    };

    struct Leg {
        std::string name;
        int baseId = 0;
        int serverId = 0;
        std::string motorType;
        std::vector<int> deviceIds;
    };

    struct Motor {
        std::string observerIp;
        int maxCanDevice = 0;
        int motorsPerLeg = 0;
        std::vector<MotorType> types;
        std::vector<Leg> legs;
    };
    struct Logger {
        std::string path;
        std::string level;
        int maxSize = 0;
        int rotation = 3;
    };

    struct DataLogger {
        bool enabled = true;
        int downsampleEvery = 5;
        int ringBufferCapacity = 256;
    };

    // --- Driver Station -------------------------------------------------------
    struct DriverStation {
        int udpPort = 61123;
        std::map<std::string, std::string> buttons;
    };

    // --- Access ---------------------------------------------------------------
    static Config &instance();
    static void init(const std::string &yamlPath);

    const Mqtt &mqtt() const { return m_mqtt; }
    const Udp &udp() const { return m_udp; }
    const Imu &imu() const { return m_imu; }
    const Motor &motor() const { return m_motor; }
    const Logger &logger() const { return m_logger; }
    const DataLogger &dataLogger() const { return m_dataLogger; }
    const DriverStation &driverStation() const { return m_driverStation; }

    /** Find a motor-type descriptor by name (e.g. "DM8009"). */
    const MotorType *findMotorType(const std::string &name) const;

    /** Find a leg descriptor by name (e.g. "left" or "right"). */
    const Leg *findLeg(const std::string &name) const;

private:
    Config() = default;
    void load(const std::string &yamlPath);

    Mqtt m_mqtt;
    Udp m_udp;
    Imu m_imu;
    Motor m_motor;
    Logger m_logger;
    DataLogger m_dataLogger;
    DriverStation m_driverStation;

    static std::string s_yamlPath;
};
