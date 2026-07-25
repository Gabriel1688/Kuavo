# Performance Tuning

## 1. Replacing `std::chrono` with Raw `clock_gettime` in Hot Loops

### Problem

Perf profiling revealed ~8% CPU overhead from `std::chrono` operations in the real-time control loops:

```
Overhead  Symbol
1.94%     std::chrono::duration<long, nano>::count
1.38%     std::chrono::__duration_cast_impl<milliseconds, micro, ...>::__cast
1.19%     std::chrono::operator-<long, nano, long, nano>
0.83%     std::chrono::operator- (duration)
0.73%     std::chrono::time_point<steady_clock, nano>::time_since_epoch
0.69%     std::chrono::operator- (time_point)
0.62%     std::chrono::operator-<steady_clock, ...>
0.54%     std::chrono::duration<long, milli>::count
```

The `std::chrono` entries dominate because the sampler is hitting lots of tiny
inlined time/duration operations in tight timing/control loops.

### Root Cause

`std::chrono` on libstdc++ generates multi-instruction sequences for every tiny
operation (`duration_cast`, `operator-`, `count()`, `time_since_epoch()`). In
the 400 Hz control loops these accumulate because:

- Each `steady_clock::now()` wraps `clock_gettime` but adds template
  instantiation overhead.
- Each `duration_cast<milliseconds>` is a separate divide + truncate.
- Each `operator-` on `time_point`/`duration` is another function call.
- The compiler often cannot inline through the template layers.

### Solution

Replace `std::chrono` calls in hot paths with raw
`clock_gettime(CLOCK_MONOTONIC)`, returning `uint64_t` nanoseconds (or
microseconds). On Linux this is a **vDSO call** (~20 ns, no syscall overhead)
and compiles to a single function call plus trivial integer arithmetic,
eliminating all the chrono template machinery.

The `Composer` class already followed this pattern correctly via
`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` and the
`get_monotonic_ns()` helper from `mercury_shm.h`.

### Helper Functions

Two static helpers were introduced to keep call sites clean:

```cpp
// Nanosecond resolution (used in ControlledSubsystemBase)
static uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

// Microsecond resolution (used in TimedRobot)
static uint64_t monotonic_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}
```

The project-wide helper `mercury::get_monotonic_ns()` in `include/mercury_shm.h`
is equivalent and used in `Motor.cpp`.

### Changes

| File | Change | Impact |
|------|--------|--------|
| `lib/robot/ControlledSubsystemBase.h` | Replaced 3x `steady_clock::now()` + `duration_cast` per iteration with `monotonic_ns()` (`uint64_t`) | **Biggest win** -- 400 Hz leg thread |
| `lib/robot/TimedRobot.h` | Replaced `std::chrono::microseconds` with `uint64_t` in `Callback` class; added `monotonic_us()` | 100 Hz main loop |
| `lib/robot/TimedRobot.cpp` | Main loop uses `monotonic_us()` instead of `steady_clock::now()` + `duration_cast` | 100 Hz main loop |
| `lib/motor/Motor.cpp` | Replaced `steady_clock::now()` with `mercury::get_monotonic_ns()` in mutex-wait instrumentation | ~4800 calls/sec (12 motors x 400 Hz) |
| `lib/motor/Motor.h` | Changed `m_lastSendTime` from `steady_clock::time_point` to `uint64_t m_lastSendTimeNs` | Removes `<chrono>` dependency |

### Before / After (ControlledSubsystemBase::Run -- 400 Hz)

**Before** (3 chrono calls per iteration, each expanding to multiple operations):

```cpp
auto next_wake = std::chrono::steady_clock::now();
static constexpr auto PERIOD = std::chrono::microseconds(2500);

while (m_entryThreadRunning) {
    auto now = std::chrono::steady_clock::now();
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(next_wake - now);
    int timeout_ms = remaining.count() > 0 ? static_cast<int>(remaining.count()) : 0;
    // ...
    now = std::chrono::steady_clock::now();
    if (now >= next_wake) { /* ... */ }
}
```

**After** (plain integer compare, single vDSO call each):

```cpp
static constexpr uint64_t PERIOD_NS = 2'500'000ULL;  // 2.5 ms = 400 Hz
uint64_t next_wake_ns = monotonic_ns();

while (m_entryThreadRunning) {
    uint64_t now_ns = monotonic_ns();
    int timeout_ms = 0;
    if (next_wake_ns > now_ns) {
        timeout_ms = static_cast<int>((next_wake_ns - now_ns + 999'999ULL) / 1'000'000ULL);
    }
    // ...
    now_ns = monotonic_ns();
    if (now_ns >= next_wake_ns) { /* ... */ }
}
```

### Guidelines

- **Hot loops (>100 Hz):** Use `clock_gettime(CLOCK_MONOTONIC)` with `uint64_t`
  nanoseconds or microseconds. Follow the `Composer` / `ControlledSubsystemBase`
  pattern.
- **Cold paths (reconnect timers, startup, etc.):** `std::chrono` is fine --
  readability matters more than a few extra instructions when called once per
  second.
- **Sleep:** Prefer `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`
  for periodic threads (absolute wakeup avoids drift). The Composer thread is
  the reference implementation.
- **Timestamps for logging / telemetry:** Use `mercury::get_monotonic_ns()`
  from `include/mercury_shm.h`.

---

## 2. Full Perf Profile Reference (Kuavo DSO, `cpu-clock`)

Captured via `perf record` / `perf report --stdio --dsos=Kuavo --no-children`.
12K samples, event `cpu-clock:pppH`.

### Top Functions (>= 0.10%)

```
Overhead  Symbol
────────  ──────────────────────────────────────────────────────
  1.10%   std::less<int>::operator()
  0.86%   Legged::controllerPeriodic
  0.76%   Motor::updateState
  0.60%   mercury::Composer::compose_cycle
  0.54%   std::__atomic_float<double>::load
  0.39%   Motor::parseMotorStateData
  0.36%   Motor::setMitControl
  0.35%   UdpServer::run
  0.32%   std::_Function_base::_M_empty
  0.29%   std::__shared_ptr<Motor>::get
  0.28%   std::atomic<bool>::load
  0.24%   UdpServer::dispatchMessage
  0.24%   mercury::SPSCRingBuffer<BatchLogRecord,256>::push
  0.24%   spdlog::pattern_formatter::format
  0.24%   std::_Rb_tree<...CANStorage...>::_M_lower_bound
  0.24%   mercury::SPSCRingBuffer<BatchLogRecord,256>::pop
  0.23%   HAL_WriteCANPacket
  0.20%   ImuReader::run
  0.19%   std::forward<unsigned char const*>
  0.19%   std::lock_guard<std::mutex>::lock_guard
  0.18%   UdpServer::sendMsg
  0.18%   std::_Function_base::_Base_manager<...Motor::callback bind...>::_M_clone
  0.18%   std::_Rb_tree<int,int>::_M_begin
  0.17%   doubleToUint
  0.17%   std::__atomic_float<double>::store
  0.17%   std::vector<shared_ptr<Motor>>::operator[]
  0.16%   mercury::SourceDoubleBuffer<MotorGroupStageData>::publish
  0.16%   std::_Rb_tree<...client_observer_t...>::_M_lower_bound
  0.16%   uintToDouble
  0.15%   Motor::callback
  0.15%   std::_Function_handler<...Motor::callback bind...>::_M_invoke
  0.15%   std::atomic<bool>::operator bool
  0.14%   CAN::writePacket
  0.13%   Motor::parseMotorParamData
  0.13%   __gthread_active_p
  0.12%   std::_Function_handler<...Motor::callback bind...>::_M_manager
  0.11%   Motor::sendMessage
  0.11%   Motor::getLastUpdateTime
  0.11%   clock_gettime@plt
  0.11%   fmt::v11::detail::buffer<char>::append<char>
  0.10%   Motor::getVelocity
  0.10%   __gthread_mutex_lock
```

### Observations

1. **`std::less<int>::operator()` at 1.10%** -- This is the red-black tree
   comparator for `std::map<int, ...>` lookups. The CAN dispatch path
   (`UdpServer::dispatchMessage` -> `std::map::find`) and motor callback
   routing hit this on every packet. Combined with the `_M_lower_bound`
   entries (~0.24% each for 3 different map types), **map lookups consume
   ~2% total**. A future optimization would be to replace hot-path
   `std::map<int, ...>` with flat arrays indexed by device ID (12 motors,
   bounded range).

2. **`std::_Function_base` / `std::function` overhead ~0.9%** --
   `_M_empty`, `_M_clone`, `_M_invoke`, `_M_manager`, ctor/dtor of
   `std::function` objects. The motor callback dispatch path creates and
   destroys `std::function` wrappers per packet. Consider replacing with a
   direct virtual call or a plain function pointer + `void*` context.

3. **`std::__atomic_float<double>::load/store` at 0.71%** -- Motor state
   reads via `atomic<double>`. This is expected and acceptable for lock-free
   motor state sharing.

4. **`std::shared_ptr` overhead ~0.5%** -- `get()`, `operator->`,
   `_M_add_ref_copy`, `_M_release` across `shared_ptr<Motor>`,
   `shared_ptr<CAN>`, `shared_ptr<CANStorage>`. The ref-count
   increment/decrement is an atomic RMW. Consider passing `Motor*` raw
   pointers in the hot loop since ownership is static.

5. **`std::vector<uint8_t>` construction ~0.4%** -- `Motor::callback`
   allocates `std::vector<uint8_t>(msg, msg + size)` on every packet
   (heap allocation + copy). Consider passing `std::span<const uint8_t>`
   or `const uint8_t*, size_t` directly to avoid the allocation.

6. **`clock_gettime@plt` + `mercury::get_monotonic_ns` at 0.19%** -- The
   residual cost of time queries after the chrono elimination. This is the
   expected floor (vDSO cost, unavoidable).

7. **Residual `std::chrono` at ~0.15% total** -- Only from cold paths
   (MQTT reconnect, `system_clock::now` in CANAPI logging). Acceptable.

### Potential Next Optimizations (by estimated impact)

| Priority | Target | Est. Savings | Status |
|----------|--------|-------------|--------|
| ~~1~~ | ~~`std::map` -> flat array for CAN dispatch~~ | ~~~2%~~ | **Done** (Section 4) |
| ~~2~~ | ~~`std::function` in callback path~~ | ~~~0.9%~~ | **Done** (Section 3) |
| 3 | `std::vector<uint8_t>` alloc in `Motor::callback` | ~0.4% | Pass `const uint8_t*, size_t` directly |
| 4 | `std::shared_ptr<Motor>` in hot loop | ~0.5% | Use raw `Motor*` (ownership is static) |

---

## 3. Replacing `std::function` with Function Pointer + Context in Callback Path

### Problem

The CAN packet dispatch path (`UdpServer::dispatchMessage`) uses
`std::function<void(const uint8_t*, size_t)>` for motor callbacks. Perf
profiling shows ~0.9% CPU spent in `std::function` machinery:

```
Overhead  Symbol
────────  ──────────────────────────────────────────────────────
  0.32%   std::_Function_base::_M_empty
  0.18%   std::_Function_base::_Base_manager<...>::_M_clone
  0.15%   std::_Function_handler<...>::_M_invoke
  0.12%   std::_Function_handler<...>::_M_manager
  0.09%   std::_Function_base::_Function_base
  0.07%   std::_Function_base::~_Function_base
  ─────
  ~0.9%   total
```

### Current Dispatch Chain

```
UDP packet arrives
  → UdpServer::run()          [epoll_wait, recvfrom]
  → dispatchMessage(frame)
      → m_subscribers.find(deviceId)                // std::map lookup
      → handler = subscriber->second.packetHandler  // COPIES std::function
      → handler(frame.data, 8)                      // invokes Motor::callback
```

The callback is a `std::bind(&Motor::callback, this, _1, _2)` stored in
`client_observer_t<uint8_t>::packetHandler` (type: `std::function`). It wraps
exactly two things: a member function pointer and a `this` pointer.

**Key facts:**

- Set once in `Motor` constructor, never reassigned.
- Copied into a local `std::function handler` on every packet to release the
  subscriber mutex before invocation.
- The copy triggers `_M_clone` (deep-copies the internal `std::bind` object)
  and the destructor triggers `_M_manager` / `~_Function_base`.

### Design Decision: Function Pointer + `void*` vs. Virtual Call

Both eliminate `std::function` overhead. Here is why function pointer + context
is the better fit for this codebase.

#### Why virtual call is a poor fit

- **Requires an interface hierarchy.** `client_observer_t<T>` is a plain
  struct template used across the codebase. Forcing `Motor` to inherit from
  an `IPacketHandler` interface pollutes a general-purpose type with a vtable.
- **Heavier indirection.** A virtual call loads the vtable pointer, then loads
  the function pointer from the vtable -- two dependent cache loads vs. one
  indirect call for a function pointer.
- **Ownership model change.** The subscriber map stores
  `client_observer_t<uint8_t>` by value. Switching to a virtual interface
  requires storing pointers, changing ownership semantics.

#### Why function pointer + context wins

The callback captures exactly one thing (a `Motor*`). A function pointer +
`void*` is the exact model:

```cpp
struct PacketCallback {
    void (*fn)(void* ctx, const uint8_t* data, size_t len);
    void* ctx;
};
```

| Criterion | `std::function` | Virtual call | Function ptr + `void*` |
|-----------|----------------|--------------|----------------------|
| Copy cost per packet | ~50-100 ns (clone + manager) | N/A (pointer copy) | ~2 ns (16 bytes memcpy) |
| Call overhead | indirect + trampoline | 2 dependent loads (vtable) | 1 indirect call |
| Object size | 32-48 bytes + possible heap | 8 bytes ptr + vtable | 16 bytes |
| Flexibility needed? | No (set once, never changes) | No | No |
| Invasiveness | Low | High (needs interface hierarchy) | Low |

**Function pointer + `void*` matches the actual semantics:** a fixed callback
with one captured pointer. It has minimal copy cost and requires no type
hierarchy changes.

### Proposed Change

#### 1. Replace the callback type in `client_observer_t`

```cpp
// include/types.h  -- before
template<typename T>
struct client_observer_t {
    std::string wantedIP = "";
    std::function<void(const T *payload, size_t size)> packetHandler = nullptr;
};

// include/types.h  -- after
template<typename T>
struct client_observer_t {
    std::string wantedIP = "";
    void (*packetHandler)(void* ctx, const T* payload, size_t size) = nullptr;
    void* packetCtx = nullptr;
};
```

#### 2. Add a static trampoline in Motor

```cpp
// lib/motor/Motor.h
static void packetTrampoline(void* ctx, const uint8_t* data, size_t len) {
    static_cast<Motor*>(ctx)->callback(data, len);
}
```

#### 3. Bind at construction

```cpp
// lib/motor/Motor.cpp  -- before
m_observer.packetHandler = std::bind(&Motor::callback, this, _1, _2);

// lib/motor/Motor.cpp  -- after
m_observer.packetHandler = &Motor::packetTrampoline;
m_observer.packetCtx     = this;
```

#### 4. Update dispatch site

```cpp
// lib/motor/UdpServer.cpp  -- before (copies std::function)
std::function<void(const uint8_t *, size_t)> handler;
// ...
handler = subscriber->second.packetHandler;
// ...
if (handler) { handler(frame.data, 8); }

// lib/motor/UdpServer.cpp  -- after (copies two pointers)
void (*handler)(void*, const uint8_t*, size_t) = nullptr;
void* handler_ctx = nullptr;
// ...
handler     = subscriber->second.packetHandler;
handler_ctx = subscriber->second.packetCtx;
// ...
if (handler) { handler(handler_ctx, frame.data, 8); }
```

### Expected Impact

- Eliminates `_M_clone`, `_M_manager`, `_M_invoke`, `_M_empty`,
  `_Function_base` ctor/dtor from the profile (~0.9%).
- The per-packet copy becomes 16 bytes (two pointers) instead of a
  deep-copy of the `std::bind` internals.
- No heap allocations in the dispatch path.
- The indirect call `fn(ctx, data, len)` compiles to a single `callq *%rax`
  instruction -- the same cost as calling any non-inlined function.

---

## 4. Replacing `std::map` with Flat Arrays for CAN Dispatch

### Problem

Perf profiling showed ~2% CPU overhead from `std::map<int, ...>` red-black tree
operations in the CAN packet dispatch hot path:

```
Overhead  Symbol
────────  ──────────────────────────────────────────────────────
  1.10%   std::less<int>::operator()
  0.24%   std::_Rb_tree<...CANStorage...>::_M_lower_bound
  0.16%   std::_Rb_tree<...client_observer_t...>::_M_lower_bound
  0.18%   std::_Rb_tree<int,int>::_M_begin
  ─────
  ~2.0%   total
```

Two maps were hot:
- `m_subscribers`: `std::map<int32_t, client_observer_t<uint8_t>>` — looked up
  on every CAN packet to find the motor callback.
- `m_deviceIPs`: `std::map<int, sockaddr_in*>` — looked up on every outbound
  `sendMsg()` call to resolve device ID → IP address.

### Root Cause

`std::map` uses a red-black tree: O(log n) comparisons per lookup, each
traversing multiple pointer-chasing tree nodes. For 12 motors the tree has
~4 levels, meaning 4 cache-unfriendly pointer dereferences per lookup. At
4800 packets/sec (12 motors × 400 Hz) inbound and a similar rate outbound,
this adds up.

### Design Decision: Flat Array vs. `std::unordered_map`

Device IDs are bounded integers in the range [1..12]. This makes flat arrays
the optimal choice:

| Criterion | `std::map` | `std::unordered_map` | Flat array |
|-----------|-----------|---------------------|------------|
| Lookup | O(log n), 4 pointer chases | O(1) amortized, hash + bucket chain | O(1), single indexed load |
| Memory | Tree nodes w/ 3 pointers each | Hash table + buckets | 13 × sizeof(element), contiguous |
| Cache behavior | Poor (scattered nodes) | Moderate (bucket chains) | Excellent (fits in 1-2 cache lines) |
| Insert cost | O(log n) + allocation | O(1) amortized + possible rehash | O(1), no allocation |
| Complexity | Medium | Medium | Trivial |

Since device IDs are small bounded integers set at startup, a flat array
indexed by device ID gives O(1) lookup with perfect cache locality and zero
allocations.

### Solution

#### 1. Replace map declarations in `UdpServer.h`

```cpp
// Before
std::map<int32_t, client_observer_t<uint8_t>> m_subscribers;
std::map<int, sockaddr_in*>                   m_deviceIPs;

// After
static constexpr int MAX_DEVICE_ID = 12;

client_observer_t<uint8_t> m_subscribers[MAX_DEVICE_ID + 1]{};
std::atomic<bool>          m_subscriberActive[MAX_DEVICE_ID + 1]{};
sockaddr_in*               m_deviceIPs[MAX_DEVICE_ID + 1]{};
```

Arrays are zero-initialized. `m_subscriberActive` uses `std::atomic<bool>`
with `memory_order_acquire`/`memory_order_release` to provide a visibility
fence between the thread that calls `subscribe()` (at startup) and the
dispatch thread that reads the subscriber.

#### 2. Update `subscribe()`

```cpp
// Before
void UdpServer::subscribe(const int32_t deviceId,
                           const client_observer_t<uint8_t> &observer) {
    std::lock_guard<std::mutex> lock(m_subscribersMtx);
    m_subscribers.insert(std::make_pair(deviceId, observer));
}

// After
void UdpServer::subscribe(const int32_t deviceId,
                           const client_observer_t<uint8_t> &observer) {
    if (deviceId < 0 || deviceId > MAX_DEVICE_ID) {
        SPDLOG_ERROR("UdpServer::subscribe: deviceId {} out of range", deviceId);
        return;
    }
    std::lock_guard<std::mutex> lock(m_subscribersMtx);
    m_subscribers[deviceId] = observer;
    m_subscriberActive[deviceId].store(true, std::memory_order_release);
}
```

#### 3. Update `bindDevicesToServer()`

```cpp
// Before (map insert)
m_deviceIPs.insert(std::make_pair(deviceId, &m_clientLeft));

// After (direct assignment with bounds check)
if (deviceId < 0 || deviceId > MAX_DEVICE_ID) return;
m_deviceIPs[deviceId] = &m_clientLeft;  // or &m_clientRight
```

#### 4. Update `getClientAddrByDeviceId()`

```cpp
// Before
auto it = m_deviceIPs.find(deviceId);
return (it != m_deviceIPs.end()) ? it->second : nullptr;

// After
if (deviceId < 0 || deviceId > MAX_DEVICE_ID) return nullptr;
return m_deviceIPs[deviceId];
```

#### 5. Update `dispatchMessage()` — the hot path

```cpp
// Before (map find + iterator check)
auto subscriber = m_subscribers.find(storage->deviceId);
if (subscriber != m_subscribers.end()) {
    handler     = subscriber->second.packetHandler;
    handler_ctx = subscriber->second.packetCtx;
}

// After (array index + atomic flag check)
auto devId = storage->deviceId;
if (devId >= 0 && devId <= MAX_DEVICE_ID &&
    m_subscriberActive[devId].load(std::memory_order_acquire)) {
    handler     = m_subscribers[devId].packetHandler;
    handler_ctx = m_subscribers[devId].packetCtx;
}
```

The unsolicited-packet path (where `deviceId` is derived from `FrameId - 0x10`)
was also updated identically, **and the `std::lock_guard` was removed** since:

- Subscribers are registered once at startup before packets flow.
- The `std::atomic<bool>` acquire/release fence ensures the dispatch thread
  sees the fully-written subscriber data.
- Eliminating the mutex from the per-packet path removes contention between
  the dispatch thread and any future subscriber management.

### Maps Intentionally Kept

Two maps in `UdpServer` were **not** converted:

- `m_frameIds`: `std::map<CANFrameId, int32_t>` — keys are 16-bit CAN frame
  IDs (not bounded to 1..12). Would need a 64K-entry array; not worthwhile.
- `m_canHandles`: `std::map<HAL_CANHandle, shared_ptr<CANStorage>>` — keys
  are opaque handles from the HAL layer, not bounded integers.

Both are accessed less frequently or under existing mutexes, so their overhead
is acceptable.

### Expected Impact

- Eliminates `std::less<int>::operator()` (1.10%), `_Rb_tree::_M_lower_bound`
  (0.24% × 2), and `_Rb_tree::_M_begin` (0.18%) from the profile (~2% total).
- Each subscriber lookup becomes a single array index + atomic flag load
  (1-2 cache line reads, no pointer chasing).
- Each device IP lookup becomes a single array index (likely already in L1).
- Removes one mutex acquisition per unsolicited packet from `dispatchMessage()`.

### Files Modified

| File | Change |
|------|--------|
| `lib/motor/UdpServer.h` | Replaced `std::map` members with flat arrays + `std::atomic<bool>` flags |
| `lib/motor/UdpServer.cpp` | Updated `subscribe()`, `bindDevicesToServer()`, `getClientAddrByDeviceId()`, `dispatchMessage()` |

---

## 5. Post-Optimization Perf Profile & Measured Results

Captured after all three optimizations (chrono, std::function, std::map) were
applied. `perf report --stdio --dsos=Kuavo --no-children`, 17K samples,
event `cpu-clock:pppH`.

### Before / After Comparison

| Category | Before (12K samples) | After (17K samples) | Reduction |
|----------|---------------------|---------------------|-----------|
| `std::chrono` total | ~8.0% | **0.0%** (eliminated) | **-8.0%** |
| Timing residual (`clock_gettime` + helpers) | 0.11% | 0.19% | +0.08% (expected) |
| `std::function` total | ~0.9% | 0.13% (EventLoop only) | **-0.8%** |
| `std::map` / `_Rb_tree` total | ~2.0% | ~1.2% | **-0.8%** |
| **Combined savings** | | | **~9.6%** |

### Optimization 1 Results: `std::chrono` Elimination

Fully eliminated from hot paths. No `std::chrono::duration`, `duration_cast`,
`time_since_epoch`, or `steady_clock` entries appear anywhere in the profile.

Residual timing cost (the irreducible floor):

```
Overhead  Symbol
────────  ──────────────────────────────────────
  0.07%   mercury::get_monotonic_ns
  0.06%   ControlledSubsystemBase<7,2,6>::monotonic_ns
  0.06%   clock_gettime@plt
  ─────
  0.19%   total (vDSO cost, unavoidable)
```

### Optimization 2 Results: `std::function` Elimination

The motor dispatch path no longer uses `std::function`. All `_M_clone`,
`_M_invoke`, `_M_manager`, and `_Function_base` ctor/dtor entries from the
motor callback path are gone.

```
Overhead  Symbol                                    Status
────────  ──────────────────────────────────────     ──────
  0.13%   std::_Function_base::_M_empty             Residual (EventLoop BooleanEvent, not motor path)
  0.09%   Motor::packetTrampoline                   New (direct fn ptr call, confirms optimization working)
  0.00%   std::_Function_base::_M_clone             Eliminated
  0.00%   std::_Function_handler<...>::_M_invoke    Eliminated
  0.00%   std::_Function_handler<...>::_M_manager   Eliminated
```

### Optimization 3 Results: `std::map` → Flat Array

The `m_subscribers` and `m_deviceIPs` map lookups are eliminated.
`_Rb_tree<..., client_observer_t<...>>::_M_lower_bound` is gone from the
profile entirely.

Remaining `std::map` overhead comes from `m_frameIds` (`std::map<CANFrameId,
int32_t>`) and `m_canHandles` (`std::map<HAL_CANHandle, shared_ptr<CANStorage>>`),
which were intentionally kept (keys not bounded to a small range):

```
Overhead  Symbol                                                Map
────────  ──────────────────────────────────────────────────     ────────────
  0.47%   std::less<int>::operator()                            All maps combined (was 1.10%)
  0.20%   _Rb_tree<int, pair<int const, int>>::_M_begin         m_frameIds
  0.18%   _Rb_tree<..., CANStorage>::_M_lower_bound             m_canHandles
  0.12%   map<int, int>::find                                   m_frameIds
  0.10%   _Rb_tree<..., CANStorage>::_S_key                     m_canHandles
  0.09%   _Rb_tree<int, pair<int const, int>>::find              m_frameIds
  0.06%   _Rb_tree<..., CANStorage>::find                        m_canHandles
  ─────
  ~1.2%   total remaining (was ~2.0%)
```

### New Top Functions (>= 0.10%)

```
Overhead  Symbol                                     Category
────────  ──────────────────────────────────────      ─────────────
  0.84%   Legged::controllerPeriodic                 Application logic
  0.73%   Motor::updateState                         Application logic
  0.71%   mercury::Composer::compose_cycle           Application logic
  0.57%   std::__atomic_float<double>::load          Lock-free motor state
  0.47%   std::less<int>::operator()                 Remaining map overhead
  0.47%   Motor::parseMotorStateData                 Application logic
  0.38%   std::atomic<bool>::load                    Lock-free flags
  0.34%   std::__shared_ptr<Motor>::get              shared_ptr overhead
  0.30%   spdlog::pattern_formatter::format          Logging
  0.28%   std::__atomic_float<double>::store         Lock-free motor state
  0.28%   mercury::SPSCRingBuffer::push              Lock-free logging
  0.27%   Motor::setMitControl                       Application logic
  0.22%   HAL_WriteCANPacket                         CAN I/O
  0.21%   UdpServer::run                             Network I/O
  0.21%   mercury::SPSCRingBuffer::pop               Lock-free logging
  0.21%   std::atomic<bool>::operator bool           Lock-free flags
  0.20%   _Rb_tree<int,int>::_M_begin                m_frameIds map
  0.18%   CAN::writePacket                           CAN I/O
  0.18%   UdpServer::dispatchMessage                 Dispatch logic
  0.18%   std::vector<shared_ptr<Motor>>::size        Motor iteration
  0.18%   uintToDouble                               Data conversion
  0.18%   _Rb_tree<...,CANStorage>::_M_lower_bound   m_canHandles map
  0.18%   std::vector<shared_ptr<Motor>>::operator[]  Motor iteration
  0.17%   _Sp_counted_base::_M_add_ref_copy          shared_ptr refcount
  0.17%   ControlledSubsystemBase::Run               Control loop
  0.17%   Motor::parseMotorParamData                 Application logic
  0.16%   ImuReader::run                             IMU I/O
  0.15%   doubleToUint                               Data conversion
  0.14%   SourceDoubleBuffer::publish                Lock-free staging
  0.14%   std::__shared_ptr_access::_M_get           shared_ptr overhead
  0.14%   std::allocator<unsigned char>::allocator    vector<uint8_t> alloc
  0.14%   std::scoped_lock::scoped_lock              Mutex overhead
  0.13%   UdpServer::sendMsg                         Network I/O
  0.13%   _Vector_impl::_Vector_impl                 vector<uint8_t> alloc
  0.13%   std::mutex::lock                           Mutex overhead
  0.13%   std::_Function_base::_M_empty              EventLoop (not motor)
  0.13%   ~__shared_count                            shared_ptr dtor
  0.12%   std::map<int,int>::find                    m_frameIds map
  0.11%   std::vector<uint8_t>::operator[]           vector<uint8_t> access
  0.11%   std::scoped_lock::~scoped_lock             Mutex overhead
  0.10%   Motor::getLastUpdateTime                   Application logic
```

---

## 6. Potential Next Optimizations

Based on the post-optimization perf profile (Section 5):

| Priority | Target | Current Cost | Approach |
|----------|--------|-------------|----------|
| 1 | `std::vector<uint8_t>` alloc in `Motor::callback` | ~0.7% (allocator + _Vector_impl + _M_range_initialize + operator[] + dtor) | Pass raw `const uint8_t*, size_t` through to `parseMotorStateData`; eliminate per-packet heap allocation |
| 2 | `std::shared_ptr<Motor>` in hot loops | ~0.7% (get + _M_get + operator-> + _M_add_ref_copy + ~__shared_count) | Use raw `Motor*` in `Composer::compose_cycle()` (ownership is static for process lifetime) |
| 3 | `m_frameIds` map (`std::map<CANFrameId, int32_t>`) | ~0.5% (less, _M_lower_bound, _M_begin, find for int,int maps) | `std::unordered_map` or bounded array if frame IDs can be mapped to a small range |

---

## 7. Memory Leak & Crash Debugging Procedure

### Tools Used

#### AddressSanitizer (ASan)
**Purpose**: Detect memory corruption, use-after-free, and uninitialized memory access

**Usage**:
```bash
cmake -B cmake-build-debug -DENABLE_ASAN=ON .
cmake --build cmake-build-debug --target Kuavo
./Kuavo
```

**Key Findings**:
- Detected heap-use-after-free in UdpServer (static singleton lifecycle issue)
- Identified uninitialized values in MqttClient constructor
- Found SEGV crashes related to SHM lifecycle management

#### Valgrind
**Purpose**: Detect memory leaks, uninitialized values, and invalid memory access

**Usage**:
```bash
cmake -B cmake-build-debug -DENABLE_ASAN=OFF .
cmake --build cmake-build-debug --target Kuavo
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=valgrind.log ./Kuavo
```

**Key Findings**:
- Uninitialized values in MqttClient::m_pubParam and m_subParam
- Memory leaks from static singletons (expected, not critical)
- No use-after-free or invalid read/write errors after fixes

#### GDB
**Purpose**: Debug crashes and examine thread states

**Usage**:
```bash
gdb ./Kuavo
(gdb) run
# When crash occurs:
(gdb) bt
(gdb) thread apply all bt
(gdb) t <thread_id>
```

**Key Findings**:
- Identified crash locations in Legged::controllerPeriodic()
- Traced thread states during shutdown
- Confirmed thread blocking in pthread_cond_wait

#### htop
**Purpose**: Monitor thread names and CPU usage

**Usage**:
```bash
htop
# Press 't' to toggle thread view
# Press 'F2' → Display options → Enable "Show custom thread names"
```

**Key Findings**:
- Verified thread names are set correctly
- Monitored thread activity during normal operation

### Issues Found & Fixed

#### 1. MQTT Client Uninitialized Values
**Problem**: MqttClient constructor didn't initialize critical members, causing Valgrind to report uninitialized values in `lws_mqtt_client_send_publish`.

**Fix**:
```cpp
MqttClient() {
    m_shutdown = false;
    m_clientId = "MqttClient";
    m_context = nullptr;
    m_newDataOccurHandler = nullptr;
    memset(&m_subParam, 0, sizeof(m_subParam));
    memset(&m_pubParam, 0, sizeof(m_pubParam));
}
```

**Location**: `lib/mqtt/MqttClient.h`

#### 2. UdpServer Static Singleton Lifecycle
**Problem**: UdpServer instances are static singletons with receive threads that never get stopped. During shutdown, the threads continue running after the UdpServer objects are destroyed, causing heap-use-after-free.

**Fix**:
- Added `~UdpServer()` to call `close()`
- Made `close()` idempotent and safe to call even if thread never started
- Added `m_threadStarted` flag to guard `pthread_join()`
- Added `m_isClosed{true}` and `m_sockfd = -1` defaults

**Location**: `lib/motor/UdpServer.h`, `lib/motor/UdpServer.cpp`

#### 3. Robot Destructor Ordering
**Problem**: `~Robot()` called `munmap()` directly without stopping leg threads first, causing SEGV when leg threads accessed unmapped SHM.

**Fix**: Changed `~Robot()` to call `detachSharedMemory()` which properly stops all threads before unmapping.

**Location**: `src/Robot.cpp`

#### 4. SHM Lifecycle Race Conditions
**Problem**: Multiple race conditions between RT leg threads and main thread during SHM attach/detach:
- `pause()/resume()` race where thread could enter `controllerPeriodic()` after `pause()` returned
- `munmap()` race where thread could load SHM pointer after `resume()` but before `munmap()`

**Fix**:
- Changed `detachSharedMemory()` to use `stopThread()` instead of `pause()/resume()`
- Fixed `pause()/controllerPeriodic()` race by setting `m_inControllerPeriodic = true` BEFORE checking `m_pauseRequested`
- Changed `attachSharedMemory()` to use `startThread()` to restart stopped threads
- Made SHM pointers atomic (`std::atomic<...>`) for safe concurrent access

**Location**: `src/Robot.cpp`, `lib/robot/ControlledSubsystemBase.h`, `src/subsystems/Legged.h`, `lib/imu/ImuReader.h`

#### 5. Thread Naming
**Problem**: Thread names were set in base class constructor before derived class was fully constructed, resulting in generic "subsystem" names instead of "left"/"right".

**Fix**:
- Removed thread naming from base class constructor
- Added `setThreadName()` method for derived classes to call after construction
- `Legged` constructor calls `setThreadName()` at the end
- `startThread()` also calls `setThreadName()` when recreating threads

**Location**: `lib/robot/ControlledSubsystemBase.h`, `src/subsystems/Legged.cpp`

### Thread Names Set

| Thread | Name | Priority | Stack Size |
|--------|------|----------|------------|
| Main | `main` | SCHED_FIFO 75 | - |
| Left leg | `left` | SCHED_FIFO 90 | 256 KB |
| Right leg | `right` | SCHED_FIFO 90 | 256 KB |
| UDP Server 0 | `udp-server-0` | SCHED_FIFO 88 | 128 KB |
| UDP Server 1 | `udp-server-1` | SCHED_FIFO 88 | 128 KB |
| IMU Reader | `imu-reader` | SCHED_FIFO 80 | 128 KB |
| MQTT Client | `mqtt-client` | SCHED_OTHER 0 | 512 KB |
| MQTT Logger | `mqtt-logger` | SCHED_OTHER 0 | 128 KB |
| Composer | `composer` | SCHED_FIFO 85 | 128 KB |

### Debugging Procedure

#### Step 1: Enable ASan
```bash
cmake -B cmake-build-debug -DENABLE_ASAN=ON .
cmake --build cmake-build-debug --target Kuavo
```

#### Step 2: Run and Collect ASan Report
```bash
./Kuavo
# When crash occurs, ASan will print detailed report
```

#### Step 3: Analyze ASan Report
- Look for "heap-use-after-free" - indicates object lifecycle issues
- Look for "SEGV" - indicates invalid memory access
- Look for "uninitialized value" - indicates missing initialization

#### Step 4: Fix Critical Issues
- Address heap-use-after-free by fixing object lifecycle
- Fix uninitialized values by properly initializing constructors
- Fix SEGV by adding bounds checks and fixing race conditions

#### Step 5: Verify with Valgrind
```bash
cmake -B cmake-build-debug -DENABLE_ASan=OFF .
cmake --build cmake-build-debug --target Kuavo
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=valgrind.log ./Kuavo
```

#### Step 6: Analyze Valgrind Report
- Check for "Conditional jump or move depends on uninitialised value(s)"
- Check for "Syscall param ... points to uninitialised byte(s)"
- Memory leaks marked "still reachable" are typically singletons (not critical)

#### Step 7: Verify Thread Names
```bash
htop
# Press 't' for thread view
# Verify thread names match expected values
```

### Best Practices

#### Memory Management
1. **Initialize all members in constructors** - especially pointers and structs
2. **Stop threads before destroying objects** they reference
3. **Use atomic operations for shared pointers** accessed by multiple threads
4. **Join threads before unmapping memory** they might access
5. **Make destructors idempotent** - safe to call multiple times

#### Thread Safety
1. **Use `std::atomic` for flags** shared between threads
2. **Set thread names immediately after creation** for better debugging
3. **Use `pthread_setname_np()` with short names** (16 char limit)
4. **Guard `pthread_join()` on flags** to avoid joining unstarted threads
5. **Set thread priority after creation** using `pthread_setschedparam()`

#### Static Singletons
1. **Stop threads in destructor** - static destruction order is undefined
2. **Make cleanup explicit** - call shutdown methods before exit
3. **Use `std::atomic` for state flags** - thread-safe without locks
4. **Handle re-creation safely** - guard on running state

#### Shutdown Sequence
1. **Stop all threads that touch shared memory** before unmapping
2. **Clear pointers after stopping threads** - prevent stale references
3. **Shutdown dependent services first** (e.g., Logger before Composer)
4. **Use `pthread_join()` instead of flags** for clean shutdown
5. **Handle signal handlers carefully** - avoid `std::exit()` from signal handlers

### Performance Considerations

#### Thread Priorities
- **Main loop**: SCHED_FIFO 75 (100 Hz, 10 ms)
- **Composer**: SCHED_FIFO 85 (400 Hz, 2.5 ms)
- **UDP Servers**: SCHED_FIFO 88 (motor communication)
- **Leg threads**: SCHED_FIFO 90 (highest priority)
- **IMU Reader**: SCHED_FIFO 80 (sensor data)
- **MQTT threads**: SCHED_OTHER 0 (non-RT)

#### Stack Sizes
- **Leg threads**: 256 KB (largest, for motor control logic)
- **MQTT Client**: 512 KB (largest, for libwebsockets)
- **Others**: 128 KB (standard for RT threads)

#### Memory Allocation
- **SHM**: POSIX shared memory for inter-process communication
- **Ring buffers**: Lock-free SPSC queues for staging data
- **Static singletons**: Expected to leak at exit (normal)
- **Thread-local**: Minimal allocation in hot paths

### Verification Checklist

- [ ] ASan reports no heap-use-after-free
- [ ] ASan reports no SEGV crashes
- [ ] ASan reports no uninitialized value errors
- [ ] Valgrind reports no invalid read/write
- [ ] Valgrind reports no use-after-free
- [ ] Valgrind reports no uninitialized value errors
- [ ] Thread names visible in htop
- [ ] Process shuts down cleanly (no hangs)
- [ ] SHM attach/detach works repeatedly
- [ ] All threads have correct priorities
| 4 | `m_canHandles` map (`std::map<HAL_CANHandle, shared_ptr<CANStorage>>`) | ~0.4% (_M_lower_bound, _S_key, find for CANStorage maps) | `std::unordered_map` for O(1) amortized lookup |
