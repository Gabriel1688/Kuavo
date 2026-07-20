#pragma once
/**
 * @file MotorParamCache.h
 * @brief Atomic cache for slow-changing motor parameters (bus voltage,
 *        bus current, reflected rotor inertia).
 *
 * Written at ~10 Hz by UdpServer parameter-response callbacks.
 * Read at 400 Hz by the Composer thread via memory_order_acquire.
 */

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace mercury {

static_assert(std::atomic<double>::is_always_lock_free,
              "std::atomic<double> must be lock-free for real-time use");

class MotorParamCache {
public:
    static constexpr size_t NUM_MOTORS = 12;

    // ── Accessors (Composer reads) ──────────────────────────────────

    double load_bus_voltage(size_t idx) const {
        return bus_voltage_[idx].load(std::memory_order_acquire);
    }
    double load_bus_current(size_t idx) const {
        return bus_current_[idx].load(std::memory_order_acquire);
    }
    double load_reflected_rotor_inertia(size_t idx) const {
        return reflected_rotor_inertia_[idx].load(std::memory_order_acquire);
    }

    // ── Mutators (UdpServer callback writes) ────────────────────────

    void store_bus_voltage(size_t idx, double val) {
        bus_voltage_[idx].store(val, std::memory_order_release);
    }
    void store_bus_current(size_t idx, double val) {
        bus_current_[idx].store(val, std::memory_order_release);
    }
    void store_reflected_rotor_inertia(size_t idx, double val) {
        reflected_rotor_inertia_[idx].store(val, std::memory_order_release);
    }

private:
    std::atomic<double> bus_voltage_[NUM_MOTORS]{};
    std::atomic<double> bus_current_[NUM_MOTORS]{};
    std::atomic<double> reflected_rotor_inertia_[NUM_MOTORS]{};
};

} // namespace mercury
