/* Host test of firmware/rp2040/src/control.c against a simple motor model. Run by test_firmware.py.
 * Exit code 0 = all checks passed. */
#include <math.h>
#include <stdio.h>

#include "control.h"

static int failures;
#define CHECK(cond, ...)                         \
  do {                                           \
    if (!(cond)) {                               \
      printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                       \
      printf("\n");                              \
      failures++;                                \
    }                                            \
  } while (0)

static rl_config_t default_cfg(void) {
  rl_config_t c = {3960.0f, 0.05f, 0.3f, 0.085f, 0.04f, 0.95f, 250, 4, 0};
  return c;
}

/* JGB37-520-like wheel: first order, no-load 11.52 rad/s at duty 1 minus a load factor, static
 * friction below |duty| 0.03, quantised by the encoder. */
typedef struct {
  double w;       /* rad/s */
  double angle;   /* rad */
  double load;    /* 0..1 fraction of speed lost to load */
} motor_t;

static void motor_step(motor_t *m, double duty, double dt) {
  const double tau = 0.06, w_max = 11.52;
  double w_ss = fabs(duty) < 0.03 ? 0.0 : (duty - copysign(0.03, duty)) * w_max * (1.0 - m->load);
  m->w += (w_ss - m->w) * dt / tau;
  m->angle += m->w * dt;
}

static double run(motor_t *m, const rl_config_t *cfg, float target, double seconds, int *sat_count,
                  vel_estimator_t *est, wheel_ctrl_t *ctrl, uint32_t *t_us) {
  const double dt = 0.01;
  const int sub = 10;
  double w_sum = 0;
  int n = 0;
  for (int k = 0; k < (int)(seconds / dt); ++k) {
    int32_t count = (int32_t)floor(m->angle / (2 * M_PI) * cfg->counts_per_rev);
    float v = vel_estimator_update(est, count, *t_us, cfg->vel_window, cfg->counts_per_rev);
    bool sat;
    float duty = wheel_ctrl_update(ctrl, cfg, target, v, (float)dt, &sat);
    if (sat && sat_count) (*sat_count)++;
    for (int s = 0; s < sub; ++s) motor_step(m, duty, dt / sub);
    *t_us += 10000;
    if (k * dt > seconds - 0.3) {
      w_sum += m->w;
      n++;
    }
  }
  return w_sum / n;  /* mean true speed over the last 0.3 s */
}

static void test_tracking(void) {
  rl_config_t cfg = default_cfg();
  /* loads up to 20 %: 8 rad/s needs duty ~0.9 there, still below max_duty */
  for (int load = 0; load <= 2; ++load) {
    motor_t m = {0, 0, load * 0.1};
    vel_estimator_t est;
    wheel_ctrl_t ctrl;
    vel_estimator_reset(&est);
    wheel_ctrl_reset(&ctrl);
    uint32_t t = 0;
    const float targets[] = {3.0f, 8.0f, -5.0f, 0.5f};
    for (int i = 0; i < 4; ++i) {
      double w = run(&m, &cfg, targets[i], 1.5, NULL, &est, &ctrl, &t);
      CHECK(fabs(w - targets[i]) < 0.05 + 0.02 * fabs(targets[i]),
            "load %.2f target %.2f reached %.3f", m.load, targets[i], w);
    }
    double w = run(&m, &cfg, 0.0f, 1.0, NULL, &est, &ctrl, &t);
    CHECK(fabs(w) < 0.05, "stop: %.3f rad/s", w);
    CHECK(ctrl.integral == 0.0f, "integral not cleared at rest: %f", ctrl.integral);
  }
}

static void test_windup(void) {
  /* after a long saturation (unreachable target under heavy load) the step to a reachable target
   * must look like the same step from a fresh controller: no wound-up integral to unwind */
  rl_config_t cfg = default_cfg();
  vel_estimator_t est;
  wheel_ctrl_t ctrl;
  uint32_t t = 0;

  motor_t fresh = {0, 0, 0.5};
  vel_estimator_reset(&est);
  wheel_ctrl_reset(&ctrl);
  run(&fresh, &cfg, 5.0f, 2.0, NULL, &est, &ctrl, &t);   /* settle at 5 rad/s */
  double w_ref = run(&fresh, &cfg, 3.0f, 0.6, NULL, &est, &ctrl, &t);

  motor_t m = {0, 0, 0.5};
  vel_estimator_reset(&est);
  wheel_ctrl_reset(&ctrl);
  int sat = 0;
  run(&m, &cfg, 11.0f, 2.0, &sat, &est, &ctrl, &t);
  CHECK(sat > 100, "expected saturation, got %d cycles", sat);
  CHECK(fabsf(ctrl.integral) <= cfg.max_duty, "integral %f beyond max_duty", ctrl.integral);
  double w = run(&m, &cfg, 3.0f, 0.6, NULL, &est, &ctrl, &t);
  CHECK(w - w_ref < 0.3, "after saturation %.3f rad/s, from 5 rad/s %.3f: integral wound up", w, w_ref);
}

static void test_estimator(void) {
  vel_estimator_t e;
  vel_estimator_reset(&e);
  /* 1 rev/s = 3960 counts/s, starting near INT32_MAX to cross the wrap */
  int32_t c = 2147483647 - 100;
  uint32_t t = 4294967295u - 25000; /* and the 32-bit microsecond timer wrap */
  float v = 0;
  for (int k = 0; k < 10; ++k) {
    v = vel_estimator_update(&e, c, t, 4, 3960.0f);
    c = (int32_t)((uint32_t)c + 40);  /* 40 counts per 10 ms = 4000 counts/s */
    t += 10000;
  }
  double expect = 4000.0 / 3960.0 * 2 * M_PI;
  CHECK(fabs(v - expect) < 1e-3, "velocity %f, expected %f", v, expect);
  vel_estimator_reset(&e);
  CHECK(vel_estimator_update(&e, 5, 0, 4, 3960.0f) == 0.0f, "first sample must give 0");
}

static void test_sanitize(void) {
  rl_config_t c = default_cfg();
  CHECK(!config_sanitize(&c), "valid config changed");
  c.max_duty = 3.0f;
  c.kp = NAN;
  c.vel_window = 0;
  c.cmd_timeout_ms = 5;
  CHECK(config_sanitize(&c), "invalid config not flagged");
  CHECK(c.max_duty == 1.0f && c.kp == 0.0f && c.vel_window == 1 && c.cmd_timeout_ms == 20,
        "clamping wrong");
}

int main(void) {
  test_tracking();
  test_windup();
  test_estimator();
  test_sanitize();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all control checks passed\n");
  return 0;
}
