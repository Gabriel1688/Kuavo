#include "DSControlWord.h"
#include "DriverStation.h"

DSControlWord::DSControlWord() {
    HAL_GetControlWord(&m_controlWord);
}

bool DSControlWord::isEnabled() const {
    return m_controlWord.enabled && m_controlWord.dsAttached;
}

bool DSControlWord::isDisabled() const {
    return !(m_controlWord.enabled && m_controlWord.dsAttached);
}

bool DSControlWord::isEStopped() const {
    return m_controlWord.eStop;
}

bool DSControlWord::isAutonomous() const {
    return m_controlWord.autonomous;
}

bool DSControlWord::isAutonomousEnabled() const {
    return m_controlWord.autonomous && m_controlWord.enabled && m_controlWord.dsAttached;
}

bool DSControlWord::isTeleop() const {
    return !(m_controlWord.autonomous || m_controlWord.test);
}

bool DSControlWord::isTeleopEnabled() const {
    return !m_controlWord.autonomous && !m_controlWord.test && m_controlWord.enabled && m_controlWord.dsAttached;
}

bool DSControlWord::isTest() const {
    return m_controlWord.test;
}

bool DSControlWord::isDSAttached() const {
    return m_controlWord.dsAttached;
}