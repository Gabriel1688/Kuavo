#include "Utility.h"
#include <cstring>
#include <iomanip>
#include <sstream>

namespace utility {
double limitMinMax(double x, double min, double max) {
    return std::max(min, std::min(x, max));
}

uint16_t doubleToUint(double x, double x_min, double x_max, int bits) {
    x = limitMinMax(x, x_min, x_max);
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

std::array<uint8_t, 4> floatToUint8s(float value) {
    std::array<uint8_t, 4> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

double uintToDouble(uint16_t x, double min, double max, int bits) {
    double span = max - min;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    return data_norm * span + min;
}

float uint8sToFloat(const std::array<uint8_t, 4> &bytes) {
    float value;
    std::memcpy(&value, bytes.data(), sizeof(float));
    return value;
}

uint32_t uint8sToUint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    uint32_t value;
    uint8_t bytes[4] = {byte1, byte2, byte3, byte4};
    std::memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}
}// namespace utility
