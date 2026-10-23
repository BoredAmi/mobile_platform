#include "control.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530718f
#define STOPPED_RAD_S 0.2f  /* below this a wheel with a zero target counts as stopped */

void vel_estimator_reset(vel_estimator_t *e) {
  memset(e, 0, sizeof(*e));
}

float vel_estimator_update(vel_estimator_t *e, int32_t count, uint32_t t_us, uint8_t window,
                           float counts_per_rev) {
  const uint8_t size = RL_MAX_VEL_WINDOW + 1;
  e->head = (uint8_t)((e->head + 1) % size);
  e->count[e->head] = count;
  e->t_us[e->head] = t_us;
  if (e->filled < size) {
    e->filled++;
  }
  if (window < 1) {
    window = 1;
  }
  if (window > e->filled - 1) {
    window = (uint8_t)(e->filled - 1);
  }
  if (window == 0) {
    return 0.0f;
  }
  uint8_t old = (uint8_t)((e->head + size - window) % size);
  int32_t dc = (int32_t)((uint32_t)count - (uint32_t)e->count[old]);  /* wrap-safe */
  uint32_t dt_us = t_us - e->t_us[old];
  if (dt_us == 0 || counts_per_rev <= 0.0f) {
    return 0.0f;
  }
  return (float)dc * (TWO_PI / counts_per_rev) / ((float)dt_us * 1e-6f);
}

void wheel_ctrl_reset(wheel_ctrl_t *c) {
  c->integral = 0.0f;
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float wheel_ctrl_update(wheel_ctrl_t *c, const rl_config_t *cfg, float target, float measured,
                        float dt, bool *saturated) {
  *saturated = false;
  if (target == 0.0f && fabsf(measured) < STOPPED_RAD_S) {
    c->integral = 0.0f;
    return 0.0f;
  }
  float ff = cfg->kff * target;
  if (target > 0.0f) {
    ff += cfg->deadband_duty;
  } else if (target < 0.0f) {
    ff -= cfg->deadband_duty;
  }
  float err = target - measured;
  float p = cfg->kp * err;

  /* conditional integration: no further wind-up while the output is saturated in the direction
   * the error pushes it */
  float candidate = c->integral + cfg->ki * err * dt;
  float out = ff + p + candidate;
  bool winding_up = (out > cfg->max_duty && err > 0.0f) || (out < -cfg->max_duty && err < 0.0f);
  if (!winding_up) {
    c->integral = clampf(candidate, -cfg->max_duty, cfg->max_duty);
  }
  out = ff + p + c->integral;
  *saturated = fabsf(out) >= cfg->max_duty;
  return clampf(out, -cfg->max_duty, cfg->max_duty);
}

bool config_sanitize(rl_config_t *cfg) {
  rl_config_t before = *cfg;
  if (!(cfg->counts_per_rev > 1.0f) || !isfinite(cfg->counts_per_rev)) cfg->counts_per_rev = 3960.0f;
  cfg->kp = isfinite(cfg->kp) ? clampf(cfg->kp, 0.0f, 10.0f) : 0.0f;
  cfg->ki = isfinite(cfg->ki) ? clampf(cfg->ki, 0.0f, 100.0f) : 0.0f;
  cfg->kff = isfinite(cfg->kff) ? clampf(cfg->kff, 0.0f, 1.0f) : 0.0f;
  cfg->deadband_duty = isfinite(cfg->deadband_duty) ? clampf(cfg->deadband_duty, 0.0f, 0.5f) : 0.0f;
  cfg->max_duty = isfinite(cfg->max_duty) ? clampf(cfg->max_duty, 0.0f, 1.0f) : 0.0f;
  if (cfg->cmd_timeout_ms < 20) cfg->cmd_timeout_ms = 20;
  if (cfg->cmd_timeout_ms > 2000) cfg->cmd_timeout_ms = 2000;
  if (cfg->vel_window < 1) cfg->vel_window = 1;
  if (cfg->vel_window > RL_MAX_VEL_WINDOW) cfg->vel_window = RL_MAX_VEL_WINDOW;
  cfg->flags &= 0x1F;
  return memcmp(&before, cfg, sizeof(before)) != 0;
}
