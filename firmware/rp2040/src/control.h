/* Hardware-independent wheel control: velocity estimate and PI + feed-forward speed controller.
 * Pure C, also compiled and tested on the PC (firmware/test). */
#ifndef JGB_CONTROL_H
#define JGB_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "rover_link.h"

/* Velocity from the count difference over the last `window` samples. */
typedef struct {
  int32_t count[RL_MAX_VEL_WINDOW + 1];
  uint32_t t_us[RL_MAX_VEL_WINDOW + 1];
  uint8_t head;    /* index of the newest sample */
  uint8_t filled;  /* samples stored, up to RL_MAX_VEL_WINDOW + 1 */
} vel_estimator_t;

void vel_estimator_reset(vel_estimator_t *e);
/* Add a sample and return the velocity in rad/s (0 until two samples exist). */
float vel_estimator_update(vel_estimator_t *e, int32_t count, uint32_t t_us, uint8_t window,
                           float counts_per_rev);

typedef struct {
  float integral;  /* integral term, in duty */
} wheel_ctrl_t;

void wheel_ctrl_reset(wheel_ctrl_t *c);
/* One controller step. target / measured in rad/s, dt in s. Returns the duty (-max..max) and sets
 * *saturated when the output hit the limit. A zero target with the wheel (nearly) stopped returns
 * exactly 0 and clears the integral, so a parked robot does not hum. */
float wheel_ctrl_update(wheel_ctrl_t *c, const rl_config_t *cfg, float target, float measured,
                        float dt, bool *saturated);

/* Clamp a config received from the host to safe ranges; returns true if it was changed. */
bool config_sanitize(rl_config_t *cfg);

#endif /* JGB_CONTROL_H */
