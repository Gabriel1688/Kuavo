#pragma once

#include <array>
#include <stdint.h>
#include <string>

namespace utility {
#ifdef __cplusplus
extern "C" {
#endif
double limitMinMax(double x, double min, double max);
uint16_t doubleToUint(double x, double x_min, double x_max, int bits);
double uintToDouble(uint16_t x, double min, double max, int bits);
std::array<uint8_t, 4> floatToUint8s(float value);
float uint8sToFloat(const std::array<uint8_t, 4> &bytes);
uint32_t uint8sToUint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);
#ifdef __cplusplus
}// extern "C"
#endif
}// namespace utility