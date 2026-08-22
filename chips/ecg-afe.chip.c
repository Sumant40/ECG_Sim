/*
 * ECG electrode + AFE custom chip for Wokwi.
 *
 * This version intentionally keeps the signal chain simple and deterministic:
 * it generates a Lead-II ECG waveform directly in volts, adds small wander/noise,
 * and outputs it centered around mid-rail. This avoids the instability of the
 * previous filter-heavy implementation while still giving a realistic ECG shape.
 */

#include "wokwi-api.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLE_RATE_HZ   500.0f
#define SAMPLE_PERIOD_US 2000
#define TWO_PI           6.28318530718f
#define VCC              3.3f
#define MID_RAIL         1.65f

typedef struct {
  pin_t out;
  pin_t hr_in;
  pin_t wander_in;
  pin_t leadoff_in;
  uint32_t attr_hr;
  uint32_t attr_wander;
  uint32_t attr_noise;
  uint32_t attr_mains;
  uint32_t attr_gain;
  uint32_t attr_contact;
  uint32_t attr_cmrr;
  float ecg_phase;
  float resp_phase;
  float motion_phase;
  float mains_phase;
  uint32_t rng;
} chip_state_t;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float wrapped_delta(float phase, float center) {
  float d = phase - center;
  if (d >  0.5f) d -= 1.0f;
  if (d < -0.5f) d += 1.0f;
  return d;
}

static float gaussian(float phase, float center, float sigma, float amp_v) {
  float d = wrapped_delta(phase, center);
  return amp_v * expf(-(d * d) / (2.0f * sigma * sigma));
}

static float rng_bipolar(chip_state_t *s, float amp) {
  s->rng ^= s->rng << 13;
  s->rng ^= s->rng >> 17;
  s->rng ^= s->rng << 5;
  return (((float)(s->rng & 0xffffu) / 32767.5f) - 1.0f) * amp;
}

static float lead_ii_v(float ph) {
  float p =  gaussian(ph, 0.185f, 0.028f, 0.06f);
  float q = -gaussian(ph, 0.385f, 0.009f, 0.04f);
  float r =  gaussian(ph, 0.407f, 0.010f, 0.46f);
  float s = -gaussian(ph, 0.432f, 0.012f, 0.08f);
  float t =  gaussian(ph, 0.670f, 0.070f, 0.13f);
  return p + q + r + s + t;
}

static void chip_tick(void *ud) {
  chip_state_t *s = (chip_state_t *)ud;

  if (pin_read(s->leadoff_in) == LOW) {
    float art = 0.02f * sinf(s->motion_phase) + rng_bipolar(s, 0.01f);
    pin_dac_write(s->out, clampf(MID_RAIL + art, 0.0f, VCC));
    return;
  }

  float pin_hr = pin_adc_read(s->hr_in);
  float bpm = (pin_hr > 0.02f)
                ? 40.0f + clampf(pin_hr / VCC, 0.0f, 1.0f) * 140.0f
                : clampf((float)attr_read(s->attr_hr), 40.0f, 180.0f);

  float pin_wd = pin_adc_read(s->wander_in);
  float wander_v = (pin_wd > 0.02f)
                    ? clampf(pin_wd / VCC, 0.0f, 1.0f) * 0.020f
                    : clampf((float)attr_read(s->attr_wander) / 1000.0f, 0.0f, 0.020f);

  float noise_v = clampf((float)attr_read(s->attr_noise) / 1000.0f, 0.0f, 0.010f);
  float mains_v = clampf((float)attr_read(s->attr_mains) / 1000.0f, 0.0f, 0.018f);
  float contact = clampf((float)attr_read(s->attr_contact), 0.0f, 100.0f) / 100.0f;
  float gain = clampf((float)attr_read(s->attr_gain), 100.0f, 900.0f);

  s->ecg_phase += bpm / (60.0f * SAMPLE_RATE_HZ);
  if (s->ecg_phase >= 1.0f) s->ecg_phase -= 1.0f;

  s->resp_phase += TWO_PI * 0.23f / SAMPLE_RATE_HZ;
  if (s->resp_phase >= TWO_PI) s->resp_phase -= TWO_PI;

  s->motion_phase += TWO_PI * 1.3f / SAMPLE_RATE_HZ;
  if (s->motion_phase >= TWO_PI) s->motion_phase -= TWO_PI;

  s->mains_phase += TWO_PI * 50.0f / SAMPLE_RATE_HZ;
  if (s->mains_phase >= TWO_PI) s->mains_phase -= TWO_PI;

  float ecg = lead_ii_v(s->ecg_phase) * (0.82f + 0.18f * contact);
  float wander = wander_v * sinf(s->resp_phase);
  float motion = wander_v * (1.0f - contact) * 0.18f * sinf(s->motion_phase);
  float mains = mains_v * sinf(s->mains_phase);
  float emg = rng_bipolar(s, noise_v);
  float signal_v = (ecg + wander + motion + mains + emg) * (gain / 450.0f);

  signal_v = clampf(signal_v, -0.10f, 0.52f);
  pin_dac_write(s->out, clampf(MID_RAIL + signal_v, 0.0f, VCC));
}

void chip_init(void) {
  chip_state_t *s = (chip_state_t *)malloc(sizeof(chip_state_t));
  memset(s, 0, sizeof(chip_state_t));

  s->out        = pin_init("OUT", ANALOG);
  s->hr_in      = pin_init("HR_IN", ANALOG);
  s->wander_in  = pin_init("WANDER_IN", ANALOG);
  s->leadoff_in = pin_init("LEADOFF_IN", INPUT_PULLUP);

  s->attr_hr      = attr_init("heartRate",      72);
  s->attr_wander  = attr_init("wanderAmp",      35);
  s->attr_noise   = attr_init("noiseAmp",        2);
  s->attr_mains   = attr_init("plAmp",          15);
  s->attr_gain    = attr_init("inaGain",        450);
  s->attr_contact = attr_init("contactQuality",  98);
  s->attr_cmrr    = attr_init("cmrrDb",         100);

  s->rng = 0x13579bdfu;

  printf("ECG AFE chip stable waveform output initialized\n");

  const timer_config_t cfg = { .callback = chip_tick, .user_data = s };
  timer_t t = timer_init(&cfg);
  timer_start(t, SAMPLE_PERIOD_US, true);
}