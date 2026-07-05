#include "Eigen/Core"

#include "Imu.h"
#include "spdlog/spdlog.h"
#include <functional>
#include <unistd.h>

//https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100480628/LPMS+Communication+Protocol#LP-CAN-Protocol
//https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100480688/LpSensor+Library+Documentation#Conversion-Quaternion-to-Euler-Angles-(ZYX-rotation-sequence)
//https://lp-research.atlassian.net/wiki/spaces/LKB/pages/2002288648/CAN+EDS+DBC+files

Imu::Imu() {
    // Create a client_observer_t<float> object
    client_observer_t<float> observer;
    observer.packetHandler = std::bind(&Imu::update, this, std::placeholders::_1);
    m_reader.subscribe(observer);
    m_reader.start();
}

void Imu::reset() {
}

Imu::~Imu() {
    m_reader.shutdown();
}

void Imu::reboot() {
    usleep(500);
}

void Imu::resting() {
}

bool Imu::isEnabled() {
    return m_isEnabled;
}

void Imu::update(const float *payload) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (int i = 0; i < 7; i++) {
        m_state(i) = static_cast<double>(payload[i]);
    }
}

const Eigen::Vector<double, 7> &Imu::getStates() const {
    // Pull euler + quaternion from the ImuReader float array.
    //   ImuReader::EULER_X = 9,  EULER_Y = 10, EULER_Z = 11
    //   ImuReader::QUAT_W  = 12, QUAT_X  = 13, QUAT_Y  = 14, QUAT_Z = 15
    m_state[0] = static_cast<double>(m_reader.getFloat(ImuReader::EULER_X));
    m_state[1] = static_cast<double>(m_reader.getFloat(ImuReader::EULER_Y));
    m_state[2] = static_cast<double>(m_reader.getFloat(ImuReader::EULER_Z));
    m_state[3] = static_cast<double>(m_reader.getFloat(ImuReader::QUAT_W));
    m_state[4] = static_cast<double>(m_reader.getFloat(ImuReader::QUAT_X));
    m_state[5] = static_cast<double>(m_reader.getFloat(ImuReader::QUAT_Y));
    m_state[6] = static_cast<double>(m_reader.getFloat(ImuReader::QUAT_Z));

    return m_state;
}
