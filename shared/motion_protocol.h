// motion_protocol.h
//
// MotionLink wire protocol. Single dependency-free header shared verbatim by:
//   - the UE5 plugin (MSVC, hot path)   -> sends SetpointFrame, receives FeedbackFrame
//   - tools/controller_sim (CMake, C++) -> receives SetpointFrame, sends FeedbackFrame
//
// Design constraints (see CLAUDE.md, Stage 0):
//   * Fixed-size POD structs, #pragma pack(1), little-endian on the wire.
//   * static_assert on every struct size so a layout drift breaks the build,
//     not a running rig.
//   * CRC32 (IEEE 802.3, reflected) with a table implementation, in-header.
//   * No allocations, no exceptions, no engine types. Safe to call from the
//     1 kHz worker hot path.
//
// Endianness: the wire format is little-endian. Both target platforms are
// x86-64 (UE on Windows, controller_sim on Windows), which is little-endian,
// so we serialize by raw copy. If this ever runs on a big-endian host the
// byte-swap has to be added here and nowhere else. We assert against exotic
// float layouts below; a full endianness swap is intentionally out of scope
// for the prototype.

#ifndef MOTIONLINK_MOTION_PROTOCOL_H
#define MOTIONLINK_MOTION_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// C++11 gives us static_assert and the fixed-width types above. Both MSVC and
// the CMake toolchain compiler are well past this.
#if defined(__cplusplus) && __cplusplus < 201103L
#  error "motion_protocol.h requires C++11 or newer"
#endif

// A float must be exactly 4 bytes IEEE-754 for the pose/vel arrays to be
// wire-compatible between the two builds. This holds on every mainstream x86
// compiler; assert it rather than assume it.
static_assert(sizeof(float) == 4, "motion_protocol assumes 32-bit float");

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// 'M''L''N''K' laid out so the first byte on the wire (little-endian) is 'K'.
// The exact value does not matter; a stable magic that is unlikely to appear
// in random UDP noise does.
static const uint32_t MOTION_PROTO_MAGIC   = 0x4D4C4E4Bu; // "MLNK"
static const uint8_t  MOTION_PROTO_VERSION = 1u;

// Degrees of freedom carried by every pose/velocity vector: surge, sway,
// heave, roll, pitch, yaw. Translations in metres, rotations in radians.
enum { MOTION_DOF = 6 };

// msg_type discriminator.
enum MotionMsgType {
    MOTION_MSG_SETPOINT = 1, // UE -> controller
    MOTION_MSG_FEEDBACK = 2  // controller -> UE
};

// Platform run state, reported by the controller in FeedbackFrame.state.
enum MotionState {
    MOTION_STATE_INIT    = 0,
    MOTION_STATE_ACTIVE  = 1,
    MOTION_STATE_LIMITED = 2, // watchdog tripped, ramping toward neutral
    MOTION_STATE_PARK    = 3, // parked at neutral
    MOTION_STATE_FAULT   = 4
};

// SetpointFrame.flags bitfield.
enum MotionSetpointFlags {
    MOTION_FLAG_NONE          = 0u,
    MOTION_FLAG_LIMITER_ACTIVE = 1u << 0 // limiter scaled this frame's command
};

// ---------------------------------------------------------------------------
// Wire structs
// ---------------------------------------------------------------------------

#pragma pack(push, 1)

// UE -> controller. Command for one 1 kHz tick.
//
// Field order is fixed on the wire. crc32 is last so it can be computed over
// every preceding byte in one pass (see motion_setpoint_finalize).
struct SetpointFrame {
    uint32_t magic;                 //  0 : MOTION_PROTO_MAGIC
    uint8_t  proto_ver;             //  4 : MOTION_PROTO_VERSION
    uint8_t  msg_type;              //  5 : MOTION_MSG_SETPOINT
    uint16_t flags;                 //  6 : MotionSetpointFlags
    uint32_t seq;                   //  8 : monotonically increasing per stream
    uint64_t t_tx_ns;              // 12 : sender timestamp, ns (QPC-derived)
    float    pose[MOTION_DOF];      // 20 : target pose
    float    vel[MOTION_DOF];       // 44 : target velocity
    uint32_t crc32;                 // 68 : CRC of bytes [0, 68)
};

// controller -> UE. Echo + measured state for one received setpoint.
struct FeedbackFrame {
    uint32_t magic;                     //  0 : MOTION_PROTO_MAGIC
    uint8_t  proto_ver;                 //  4 : MOTION_PROTO_VERSION
    uint8_t  msg_type;                  //  5 : MOTION_MSG_FEEDBACK
    uint8_t  state;                     //  6 : MotionState
    uint8_t  limiter_active;            //  7 : 0/1
    uint32_t seq;                       //  8 : echoed from the setpoint
    uint64_t t_rx_ns;                  // 12 : controller receive timestamp, ns
    uint64_t t_tx_ns;                  // 20 : echoed setpoint t_tx_ns
    float    measured_pose[MOTION_DOF]; // 28 : plant model output
    uint32_t fault_mask;                // 52 : bitmask, 0 == healthy
    uint32_t crc32;                     // 56 : CRC of bytes [0, 56)
};

#pragma pack(pop)

// If either assert fires, the header was edited without re-checking layout.
// Sizes are deliberately hard-coded, not sizeof-derived, so a change is loud.
static_assert(sizeof(SetpointFrame) == 72, "SetpointFrame must be 72 bytes on the wire");
static_assert(sizeof(FeedbackFrame) == 60, "FeedbackFrame must be 60 bytes on the wire");

// crc32 must be the trailing field of each frame; the finalize/validate helpers
// rely on it covering exactly the bytes that precede it.
static_assert(offsetof(SetpointFrame, crc32) == 68, "SetpointFrame.crc32 must be last");
static_assert(offsetof(FeedbackFrame, crc32) == 56, "FeedbackFrame.crc32 must be last");

// ---------------------------------------------------------------------------
// CRC32 (IEEE 802.3, reflected, poly 0xEDB88320)
// ---------------------------------------------------------------------------

// Table lives in a function-local static so this header can be included in
// multiple translation units without an ODR clash. In C++ the initialization
// is thread-safe ("magic statics"), and it is a pure read afterwards, so the
// table build never touches the hot path more than once.
inline const uint32_t* motion_crc32_table() {
    static uint32_t table[256];
    static bool initialized = false;
    if (!initialized) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        initialized = true;
    }
    return table;
}

// Standard reflected CRC32: init 0xFFFFFFFF, final XOR 0xFFFFFFFF.
inline uint32_t motion_crc32(const void* data, size_t len) {
    const uint32_t* table = motion_crc32_table();
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------
// Fill / validate helpers (POD in, POD out, no allocation)
// ---------------------------------------------------------------------------

// Stamp the constant header fields and compute crc over everything before it.
// Caller has already set seq, t_tx_ns, flags, pose, vel.
inline void motion_setpoint_finalize(SetpointFrame* f) {
    f->magic     = MOTION_PROTO_MAGIC;
    f->proto_ver = MOTION_PROTO_VERSION;
    f->msg_type  = MOTION_MSG_SETPOINT;
    f->crc32     = motion_crc32(f, offsetof(SetpointFrame, crc32));
}

inline void motion_feedback_finalize(FeedbackFrame* f) {
    f->magic     = MOTION_PROTO_MAGIC;
    f->proto_ver = MOTION_PROTO_VERSION;
    f->msg_type  = MOTION_MSG_FEEDBACK;
    f->crc32     = motion_crc32(f, offsetof(FeedbackFrame, crc32));
}

// Return 1 if magic, version, type and crc all check out; 0 otherwise.
// Never dereferences past the struct; caller guarantees at least sizeof(*f).
inline int motion_setpoint_valid(const SetpointFrame* f) {
    if (f->magic != MOTION_PROTO_MAGIC)          return 0;
    if (f->proto_ver != MOTION_PROTO_VERSION)    return 0;
    if (f->msg_type != MOTION_MSG_SETPOINT)      return 0;
    return f->crc32 == motion_crc32(f, offsetof(SetpointFrame, crc32)) ? 1 : 0;
}

inline int motion_feedback_valid(const FeedbackFrame* f) {
    if (f->magic != MOTION_PROTO_MAGIC)          return 0;
    if (f->proto_ver != MOTION_PROTO_VERSION)    return 0;
    if (f->msg_type != MOTION_MSG_FEEDBACK)      return 0;
    return f->crc32 == motion_crc32(f, offsetof(FeedbackFrame, crc32)) ? 1 : 0;
}

#endif // MOTIONLINK_MOTION_PROTOCOL_H
