# Extracting Sensor Data from Damiao Motor Feedback Frame
The Damiao motor feedback frame provides only a subset of the sensor data fields you need. Here is what you can and cannot get directly from the motor's response.

## What the Feedback Frame Contains
The Damiao feedback frame is 8 bytes with a fixed format that is the same for all three control modes (MIT, PosVel, Velocity) :

| Byte | D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
|------|------|------|------|------|------|------|------|------|
| Content | ID \| ERR<<4 | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0] \| T[11:8] | T[7:0] | T_MOS | T_Rotor |

The decoding function from the protocol specification is :
p_int  = (Data[1] << 8) | Data[2];
v_int  = (Data[3] << 4) | (Data[4] >> 4);
t_int  = ((Data[4] & 0xF) << 8) | Data[5];
position = uint_to_float(p_int, P_MIN, P_MAX, 16);  // (-12.5, 12.5)
velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);  // (-45.0, 45.0)
torque   = uint_to_float(t_int, T_MIN, T_MAX, 12);  // (-18.0, 18.0)

The openarm library implements the same decoding :
uint16_t q_uint   = (static_cast<uint16_t>(data[1]) << 8) | data[2];
uint16_t dq_uint  = (static_cast<uint16_t>(data[3]) << 4) |
(static_cast<uint16_t>(data[4]) >> 4);
uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5];
int t_mos   = static_cast<int>(data[6]);
int t_rotor = static_cast<int>(data[7]);

double recv_q   = uint_to_double(q_uint, -limits.pMax, limits.pMax, 16);
double recv_dq  = uint_to_double(dq_uint, -limits.vMax, limits.vMax, 12);
double recv_tau = uint_to_double(tau_uint, -limits.tMax, limits.tMax, 12);

## Field-by-Field Mapping
| Sensor Data Field | Available from Feedback? | How to Get It | Source Bytes |
|---|:---:|---|:---:|
| **`joint_jpos`** | ✅ **Yes** | Decode POS (16-bit) from D[1:2] using `uint_to_float(p_int, P_MIN, P_MAX, 16)`. If the motor has a gear ratio, multiply by gear ratio to get joint-level position. | D[1], D[2] |
| **`joint_jvel`** | ✅ **Yes** | Decode VEL (12-bit) from D[3:4] using `uint_to_float(v_int, V_MIN, V_MAX, 12)`. Apply gear ratio for joint-level velocity. | D[3], D[4] |
| **`motor_jpos`** | ✅ **Yes** | Same as `joint_jpos` but **without** gear ratio scaling — this is the raw motor-side position from the encoder. | D[1], D[2] |
| **`motor_jvel`** | ✅ **Yes** | Same as `joint_jvel` but **without** gear ratio scaling — raw motor-side velocity. | D[3], D[4] |
| **`jtorque`** | ✅ **Yes** | Decode T (12-bit) from D[4:5] using `uint_to_float(t_int, T_MIN, T_MAX, 12)`. | D[4], D[5] |
| **`bus_voltage`** | ❌ **No** | Not in the standard feedback frame. Must be read via the **parameter query command** (CAN ID = 0x7FF, D[2]=0x33) [3]. | — |
| **`bus_current`** | ❌ **No** | Not in the standard feedback frame. Must be read via **parameter query** or estimated from motor current and voltage. | — |
| **`motor_current`** | ❌ **No** | Not directly available. Can be **estimated** from torque using the motor's torque constant (KT): `current ≈ torque / KT`. The maximum phase current is documented per motor type [2]. | — |
| **`reflected_rotor_inertia`** | ❌ **No** | This is a **static motor parameter**, not a real-time measurement. Read it once via parameter query (RID for 转动惯量/rotor inertia) and store it. The motor identifies the rotor inertia during calibration [2]. | — |

## How to Read Parameters Not in the Feedback Frame
For bus_voltage, bus_current, and reflected_rotor_inertia, you must use the parameter query command which sends on CAN ID 0x7FF with marker byte 0x33 :
// Query parameter command format [3]:
// CAN ID = 0x7FF
// D[0:1] = motor CAN ID (little-endian)
// D[2]   = 0x33 (query marker)
// D[3]   = RID (parameter index)
// D[4:7] = 0x00

std::vector<uint8_t> pack_query_param_data(uint32_t send_can_id, int RID) {
return {static_cast<uint8_t>(send_can_id & 0xFF),
static_cast<uint8_t>((send_can_id >> 8) & 0xFF),
0x33,
static_cast<uint8_t>(RID),
0x00, 0x00, 0x00, 0x00};
}

The response is decoded differently depending on whether the RID is an integer or float parameter :
if (CanPacketDecoder::is_in_ranges(RID)) {
num = uint8s_to_uint32(data[4], data[5], data[6], data[7]);
} else {
std::array<uint8_t, 4> float_bytes = {data[4], data[5], data[6], data[7]};
num = uint8s_to_float(float_bytes);
}

Integer-type RIDs include CAN_ID (7-10), certain configuration parameters (13-16), and specific flags (35-36) .

## Complete Extraction Code
Combining the feedback frame decoding with parameter queries, here is how to populate all 9 sensor fields from a single Damiao motor:
// Within the 13-byte UDP-CAN frame [1]:
// Byte 0: DLC, Bytes 1-4: CAN ID, Bytes 5-12: CAN Data
const uint8_t* data = &frame[5];  // CAN data starts at byte 5 [1]

// --- Fields directly from feedback frame [2] ---

// D[0]: Motor ID and error status
uint8_t motor_id   = data[0] & 0x0F;
uint8_t error_code = (data[0] >> 4) & 0x0F;
// ERR: 0=disabled, 1=enabled, 8=overvoltage, 9=undervoltage,
//      A=overcurrent, B=MOS overtemp, C=coil overtemp,
//      D=comm lost, E=overload [2]

// D[1:2]: Position (16-bit mapped to [-PMAX, PMAX]) [2]
uint16_t p_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];

// D[3:4]: Velocity (12-bit mapped to [-VMAX, VMAX]) [2]
uint16_t v_uint = (static_cast<uint16_t>(data[3]) << 4) |
(static_cast<uint16_t>(data[4]) >> 4);

// D[4:5]: Torque (12-bit mapped to [-TMAX, TMAX]) [2]
uint16_t t_uint = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

// Convert to physical values [2][3]
double motor_pos = uint_to_double(p_uint, -P_MAX, P_MAX, 16);
double motor_vel = uint_to_double(v_uint, -V_MAX, V_MAX, 12);
double torque    = uint_to_double(t_uint, -T_MAX, T_MAX, 12);

// D[6]: MOS temperature, D[7]: Rotor temperature [2]
int t_mos   = static_cast<int>(data[6]);
int t_rotor = static_cast<int>(data[7]);

// --- Populate sensor data ---

// Directly available from feedback [2]
sensor.motor_jpos[joint_idx] = motor_pos;
sensor.motor_jvel[joint_idx] = motor_vel;
sensor.jtorque[joint_idx]    = torque;

// Joint-level = motor-level × gear_ratio
// Gear ratio is a static parameter read via query [2]
sensor.joint_jpos[joint_idx] = motor_pos * gear_ratio;
sensor.joint_jvel[joint_idx] = motor_vel * gear_ratio;

// --- Fields NOT in feedback — must estimate or query ---

// motor_current: estimated from torque and torque constant
// KT_OUT is a configurable parameter [2]; if set to 0.0,
// the system uses the theoretical calculated value
sensor.motor_current[joint_idx] = torque / kt_out;

// bus_voltage: read via parameter query (RID for bus voltage)
// This is a slow query — do it at startup or at low frequency
// sensor.bus_voltage[joint_idx] = query_param(can_id, RID_BUS_VOLTAGE);

// bus_current: estimated from motor_current and efficiency
// or read via parameter query if available
sensor.bus_current[joint_idx] = sensor.motor_current[joint_idx];

// reflected_rotor_inertia: static parameter from motor identification
// Read once at startup via parameter query [2]
// sensor.reflected_rotor_inertia[joint_idx] = query_param(can_id, RID_INERTIA);

## Summary: Data Availability
| Category | Fields | Source | Update Rate |
|----------|--------|--------|:-----------:|
| **Real-time from feedback** | `motor_jpos`, `motor_jvel`, `jtorque` | Feedback frame D[1:5] [2] | Every control cycle (1kHz) |
| **Derived from feedback** | `joint_jpos`, `joint_jvel` | Feedback × gear_ratio [2] | Every control cycle |
| **Estimated from feedback** | `motor_current` | `torque / KT_OUT` [2] | Every control cycle |
| **Parameter query required** | `bus_voltage`, `bus_current` | CAN ID 0x7FF, D[2]=0x33 [3] | Low frequency (1-10 Hz) |
| **Static (read once)** | `reflected_rotor_inertia` | Parameter query at startup [2] | Once |

The key insight is that the standard Damiao feedback frame only carries 3 real-time measurements (position, velocity, torque) plus 2 temperatures in its 8-byte payload . Everything else must be either derived mathematically, read via the slower parameter query interface , or treated as a static configuration value read once at startup.


