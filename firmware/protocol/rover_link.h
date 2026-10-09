/*
 * rover_link: binary protocol between the RP2040 motor/IMU board and the ROS 2 computer.
 *
 * Shared by the firmware (firmware/rp2040) and the ros2_control hardware interface
 * (ros2_ws/src/jgb_rover_hardware). firmware/tools/rover_link.py is the Python version; keep the
 * three in sync and bump RL_PROTOCOL_VERSION on any change to a message layout.
 *
 * Frame on the wire:  COBS( type:u8 | payload | crc16:u16 ) 0x00
 *   - crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type + payload, little-endian
 *   - every multi-byte field is little-endian (RP2040, x86-64 and the Pi's ARM64 all are), floats
 *     are IEEE-754 binary32
 *   - COBS removes every 0x00 from the frame, so 0x00 always ends a frame and a receiver that
 *     starts mid-stream resynchronises on the next one
 *
 * Units: rad, rad/s, m/s^2, degC, mV. Wheel signs: positive = that wheel drives the robot forward
 * (both joints rotate about +Y in the URDF, so this is also the joint sign).
 */
#ifndef ROVER_LINK_H
#define ROVER_LINK_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RL_PROTOCOL_VERSION 1

#define RL_MAX_PAYLOAD 160
/* type + payload + crc, COBS overhead (1 per 254), delimiter */
#define RL_MAX_RAW (1 + RL_MAX_PAYLOAD + 2)
#define RL_MAX_ENCODED (RL_MAX_RAW + RL_MAX_RAW / 254 + 2)

/* ---- message types: host -> MCU ---- */
#define RL_MSG_CMD_VEL 0x01     /* rl_cmd_vel_t: wheel velocity targets, closed loop on the MCU */
#define RL_MSG_CMD_PWM 0x02     /* rl_cmd_pwm_t: open-loop duty, for bench tests only */
#define RL_MSG_SET_CONFIG 0x03  /* rl_config_t; answered with RL_MSG_CONFIG_ACK */
#define RL_MSG_PING 0x04        /* no payload; answered with RL_MSG_INFO and RL_MSG_CONFIG */
#define RL_MSG_STOP 0x05        /* no payload; motors off until the next command */
#define RL_MSG_BOOTSEL 0x06     /* no payload; reboot into the USB bootloader (for flashing) */

/* ---- message types: MCU -> host ---- */
#define RL_MSG_TELEMETRY 0x81   /* rl_telemetry_t, every control cycle (100 Hz) */
#define RL_MSG_INFO 0x82        /* rl_info_t */
#define RL_MSG_CONFIG 0x83      /* rl_config_t in use (reply to PING) */
#define RL_MSG_LOG 0x84         /* text, not NUL-terminated */
#define RL_MSG_CONFIG_ACK 0x85  /* rl_config_t as applied (after clamping), reply to SET_CONFIG */

/* rl_telemetry_t.status bits */
#define RL_STATUS_IMU_OK (1u << 0)          /* last IMU read succeeded */
#define RL_STATUS_MOTORS_ENABLED (1u << 1)  /* driver enabled and a command is active */
#define RL_STATUS_MODE_VEL (1u << 2)
#define RL_STATUS_MODE_PWM (1u << 3)
#define RL_STATUS_CMD_TIMEOUT (1u << 4)     /* no command for cmd_timeout_ms: motors stopped */
#define RL_STATUS_CONFIGURED (1u << 5)      /* a SET_CONFIG was received since boot */
#define RL_STATUS_WATCHDOG_REBOOT (1u << 6) /* the last reset was the hardware watchdog */
#define RL_STATUS_BATTERY_VALID (1u << 7)   /* battery_mv is measured (ADC enabled) */
#define RL_STATUS_SATURATED_L (1u << 8)     /* left duty at max_duty */
#define RL_STATUS_SATURATED_R (1u << 9)

/* rl_config_t.flags bits */
#define RL_CFG_INVERT_LEFT_MOTOR (1u << 0)
#define RL_CFG_INVERT_RIGHT_MOTOR (1u << 1)
#define RL_CFG_INVERT_LEFT_ENCODER (1u << 2)
#define RL_CFG_INVERT_RIGHT_ENCODER (1u << 3)
#define RL_CFG_IDLE_BRAKE (1u << 4)   /* zero duty = short brake (else coast) */

#if defined(__GNUC__)
#define RL_PACKED __attribute__((packed))
#else
#error "rover_link needs packed structs (GCC / Clang)"
#endif

typedef struct RL_PACKED {
  float vel[2];  /* rad/s, [left, right] */
} rl_cmd_vel_t;

typedef struct RL_PACKED {
  float duty[2]; /* -1..1, [left, right] */
} rl_cmd_pwm_t;

typedef struct RL_PACKED {
  float counts_per_rev;  /* encoder counts per wheel revolution (x4 decoding) */
  float kp;              /* duty per rad/s of error */
  float ki;              /* duty per rad of integrated error */
  float kff;             /* duty per rad/s of target (about 1 / no-load speed) */
  float deadband_duty;   /* added in the direction of motion to overcome static friction */
  float max_duty;        /* output limit, 0..1 */
  uint16_t cmd_timeout_ms; /* stop the motors when no command arrived for this long */
  uint8_t vel_window;    /* velocity estimate over this many control cycles (1..RL_MAX_VEL_WINDOW) */
  uint8_t flags;         /* RL_CFG_* */
} rl_config_t;

#define RL_MAX_VEL_WINDOW 20

typedef struct RL_PACKED {
  uint32_t seq;           /* +1 per message; restarts at 0 after an MCU reset */
  uint32_t t_us;          /* MCU clock when the encoders were sampled */
  int32_t enc[2];         /* encoder counts, [left, right], wraps around */
  float vel[2];           /* estimated wheel velocity, rad/s */
  float target[2];        /* velocity target in use, rad/s */
  float duty[2];          /* applied duty, -1..1 */
  float gyro[3];          /* rad/s, imu_link frame */
  float accel[3];         /* m/s^2 incl. gravity, imu_link frame */
  float imu_temp_c;
  uint16_t battery_mv;    /* 0 when not measured */
  uint16_t status;        /* RL_STATUS_* */
  uint16_t rx_errors;     /* frames dropped for bad COBS / CRC / length, wraps */
  uint16_t tx_dropped;    /* telemetry frames dropped because USB was busy, wraps */
} rl_telemetry_t;

typedef struct RL_PACKED {
  uint16_t protocol_version;
  uint16_t firmware_version; /* major << 8 | minor */
  uint16_t control_hz;
  uint8_t imu_whoami;        /* MPU6050 WHO_AM_I (0x68 genuine; clones differ), 0 = no answer */
  uint8_t reset_reason;      /* 0 power-on / reset pin, 1 watchdog */
  uint32_t uptime_ms;
  uint8_t board_id[8];       /* flash unique ID, tells boards apart */
} rl_info_t;

#ifdef __cplusplus
#define RL_STATIC_ASSERT static_assert
#else
#define RL_STATIC_ASSERT _Static_assert
#endif
RL_STATIC_ASSERT(sizeof(float) == 4, "float must be binary32");
RL_STATIC_ASSERT(sizeof(rl_cmd_vel_t) == 8, "layout");
RL_STATIC_ASSERT(sizeof(rl_config_t) == 28, "layout");
RL_STATIC_ASSERT(sizeof(rl_telemetry_t) == 76, "layout");
RL_STATIC_ASSERT(sizeof(rl_info_t) == 20, "layout");

uint16_t rl_crc16(const uint8_t *data, size_t len);

/* Build a complete frame (COBS + trailing 0x00) into out (at least RL_MAX_ENCODED bytes).
 * Returns the number of bytes to send, 0 if the payload is too long. */
size_t rl_encode(uint8_t type, const void *payload, size_t len, uint8_t *out);

/* Incremental decoder: feed bytes one at a time. */
typedef struct {
  uint8_t buf[RL_MAX_ENCODED];
  size_t len;
  bool overflow;
  uint8_t frame[RL_MAX_RAW];  /* decoded type + payload (+ crc) of the last complete frame */
  uint32_t errors;            /* bad frames seen */
} rl_decoder_t;

void rl_decoder_init(rl_decoder_t *d);

/* Returns true when byte completed a valid frame; then *type and *payload / *len point into the
 * decoder (valid until the next call). Bad frames are counted in d->errors and skipped. */
bool rl_decoder_feed(rl_decoder_t *d, uint8_t byte, uint8_t *type, const uint8_t **payload,
                     size_t *len);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_LINK_H */
