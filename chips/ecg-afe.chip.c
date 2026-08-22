/*
 * ECG electrode + commercial analog front-end custom chip for Wokwi.
 *
 * Pins:
 *   HR_IN       optional 0..3.3 V control, maps to 40..180 BPM
 *   WANDER_IN   optional 0..3.3 V control, maps to 0..250 uV baseline wander
 *   LEADOFF_IN  HIGH = electrodes connected, LOW = lead off
 *   OUT         analog ECG output, biased around 1.65 V
 *
 * The model generates a normal Lead-II style P-QRS-T waveform at the electrode,
 * adds realistic but modest artifacts, then applies an ECG AFE chain:
 *   electrode -> 0.5 Hz HPF -> instrumentation gain -> 40 Hz LPF -> 50 Hz notch
 */

#include "wokwi-api.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLE_RATE_HZ 500.0f
#define SAMPLE_PERIOD_US 2000
#define TWO_PI 6.28318530718f
#define VCC 3.3f
#define MID_RAIL 1.65f
#define MAX_SWING_V 1.50f

typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float z1;
  float z2;
} biquad_t;

typedef struct {
  pin_t out;
  pin_t hr_in;
  pin_t wander_in;
  pin_t leadoff_in;

  uint32_t heart_rate_attr;
  uint32_t wander_attr;
  uint32_t noise_attr;
  uint32_t mains_attr;
  uint32_t gain_attr;
  uint32_t contact_attr;
  uint32_t cmrr_attr;

  float ecg_phase;
  float respiration_phase;
  float motion_phase;
  float mains_phase;
  uint32_t rng;

  biquad_t hpf_05;
  biquad_t lpf_40_a;
  biquad_t lpf_40_b;
  biquad_t notch_50;
} chip_state_t;

static float clampf(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static float biquad_process(biquad_t *f, float x) {
  float y = f->b0 * x + f->z1;
  f->z1 = f->b1 * x - f->a1 * y + f->z2;
  f->z2 = f->b2 * x - f->a2 * y;
  return y;
}

static void biquad_set(biquad_t *f,
                       float b0,
                       float b1,
                       float b2,
                       float a1,
                       float a2) {
  f->b0 = b0;
  f->b1 = b1;
  f->b2 = b2;
  f->a1 = a1;
  f->a2 = a2;
  f->z1 = 0.0f;
  f->z2 = 0.0f;
}

static float wrapped_delta(float phase, float center) {
  float d = phase - center;
  if (d > 0.5f) d -= 1.0f;
  if (d < -0.5f) d += 1.0f;
  return d;
}

static float gaussian(float phase, float center, float sigma, float amplitude_uV) {
  float d = wrapped_delta(phase, center);
  return amplitude_uV * expf(-(d * d) / (2.0f * sigma * sigma));
}

static float normal_lead_ii_uV(float phase) {
  /*
   * Normal sinus morphology in electrode microvolts:
   * P wave: small rounded positive wave
   * QRS: narrow Q dip, tall R spike, S dip
   * T wave: broad upright recovery wave
   */
  float p =  gaussian(phase, 0.185f, 0.030f,  120.0f);
  float q = -gaussian(phase, 0.382f, 0.010f,  140.0f);
  float r =  gaussian(phase, 0.405f, 0.010f, 1100.0f);
  float s = -gaussian(phase, 0.432f, 0.014f,  280.0f);
  float st = gaussian(phase, 0.520f, 0.060f,   18.0f);
  float t =  gaussian(phase, 0.675f, 0.075f,  320.0f);
  return p + q + r + s + st + t;
}

static float random_bipolar(chip_state_t *s, float amplitude) {
  s->rng ^= s->rng << 13;
  s->rng ^= s->rng >> 17;
  s->rng ^= s->rng << 5;
  return (((float)(s->rng & 0xffffu) / 32767.5f) - 1.0f) * amplitude;
}

static float read_bpm(chip_state_t *s) {
  float pin_v = pin_adc_read(s->hr_in);
  if (pin_v > 0.02f) {
    return 40.0f + clampf(pin_v / VCC, 0.0f, 1.0f) * 140.0f;
  }
  return clampf((float)attr_read(s->heart_rate_attr), 40.0f, 180.0f);
}

static float read_wander_uV(chip_state_t *s) {
  float pin_v = pin_adc_read(s->wander_in);
  if (pin_v > 0.02f) {
    return clampf(pin_v / VCC, 0.0f, 1.0f) * 250.0f;
  }
  return clampf((float)attr_read(s->wander_attr), 0.0f, 250.0f);
}

static void chip_tick(void *user_data) {
  chip_state_t *s = (chip_state_t *)user_data;

  if (pin_read(s->leadoff_in) == LOW) {
    s->motion_phase += TWO_PI * 1.1f / SAMPLE_RATE_HZ;
    if (s->motion_phase >= TWO_PI) s->motion_phase -= TWO_PI;

    float artifact_v = 0.18f * sinf(s->motion_phase) + random_bipolar(s, 0.08f);
    pin_dac_write(s->out, clampf(MID_RAIL + artifact_v, 0.0f, VCC));
    return;
  }

  float bpm = read_bpm(s);
  float wander_uV = read_wander_uV(s);
  float noise_uV = clampf((float)attr_read(s->noise_attr), 0.0f, 80.0f);
  float mains_uV = clampf((float)attr_read(s->mains_attr), 0.0f, 200.0f);
  float afe_gain = clampf((float)attr_read(s->gain_attr), 100.0f, 900.0f);
  float contact = clampf((float)attr_read(s->contact_attr), 0.0f, 100.0f) / 100.0f;
  float cmrr_db = clampf((float)attr_read(s->cmrr_attr), 60.0f, 120.0f);

  s->ecg_phase += bpm / (60.0f * SAMPLE_RATE_HZ);
  if (s->ecg_phase >= 1.0f) s->ecg_phase -= 1.0f;

  s->respiration_phase += TWO_PI * 0.23f / SAMPLE_RATE_HZ;
  if (s->respiration_phase >= TWO_PI) s->respiration_phase -= TWO_PI;

  s->motion_phase += TWO_PI * 1.3f / SAMPLE_RATE_HZ;
  if (s->motion_phase >= TWO_PI) s->motion_phase -= TWO_PI;

  s->mains_phase += TWO_PI * 50.0f / SAMPLE_RATE_HZ;
  if (s->mains_phase >= TWO_PI) s->mains_phase -= TWO_PI;

  float contact_loss = 1.0f - contact;
  float ecg = normal_lead_ii_uV(s->ecg_phase) * (0.85f + 0.15f * contact);
  float respiration = wander_uV * sinf(s->respiration_phase);
  float motion = wander_uV * contact_loss * 0.7f * sinf(s->motion_phase);
  float electrode_noise = random_bipolar(s, 2.0f + 35.0f * contact_loss);
  float emg_noise = random_bipolar(s, noise_uV);
  float residual_mains = mains_uV * sinf(s->mains_phase);

  float common_mode_leakage = 200000.0f
                            * powf(10.0f, -cmrr_db / 20.0f)
                            * sinf(s->mains_phase);

  float electrode_uV = ecg
                     + respiration
                     + motion
                     + electrode_noise
                     + emg_noise
                     + residual_mains
                     + common_mode_leakage;

  float hpf_uV = biquad_process(&s->hpf_05, electrode_uV);
  float amplified_uV = clampf(hpf_uV * afe_gain,
                              -MAX_SWING_V * 1000000.0f,
                               MAX_SWING_V * 1000000.0f);
  float lpf_uV = biquad_process(&s->lpf_40_b,
                  biquad_process(&s->lpf_40_a, amplified_uV));
  float output_uV = biquad_process(&s->notch_50, lpf_uV);

  float out_v = MID_RAIL + output_uV / 1000000.0f;
  pin_dac_write(s->out, clampf(out_v, 0.0f, VCC));
}

void chip_init(void) {
  chip_state_t *s = (chip_state_t *)malloc(sizeof(chip_state_t));
  memset(s, 0, sizeof(chip_state_t));

  s->out = pin_init("OUT", ANALOG);
  s->hr_in = pin_init("HR_IN", ANALOG);
  s->wander_in = pin_init("WANDER_IN", ANALOG);
  s->leadoff_in = pin_init("LEADOFF_IN", INPUT_PULLUP);

  s->heart_rate_attr = attr_init("heartRate", 72);
  s->wander_attr = attr_init("wanderAmp", 35);
  s->noise_attr = attr_init("noiseAmp", 2);
  s->mains_attr = attr_init("plAmp", 15);
  s->gain_attr = attr_init("inaGain", 450);
  s->contact_attr = attr_init("contactQuality", 98);
  s->cmrr_attr = attr_init("cmrrDb", 100);

  s->rng = 0x13579bdfu;

  /* Butterworth 2nd order HPF, fc = 0.5 Hz, fs = 500 Hz. */
  biquad_set(&s->hpf_05,
             0.99556697f, -1.99113394f, 0.99556697f,
            -1.99111429f,  0.99115360f);

  /* Butterworth 4th order LPF, fc = 40 Hz, fs = 500 Hz. */
  biquad_set(&s->lpf_40_a,
             0.00223489f, 0.00446978f, 0.00223489f,
            -1.04859958f, 0.29614036f);
  biquad_set(&s->lpf_40_b,
             1.00000000f, 2.00000000f, 1.00000000f,
            -1.32091343f, 0.63273879f);

  /* IIR notch at 50 Hz, Q = 30, fs = 500 Hz. */
  biquad_set(&s->notch_50,
             0.98963618f, -1.60126497f, 0.98963618f,
            -1.60126497f,  0.97927235f);

  printf("ECG AFE chip ready: normal Lead-II rhythm, HPF, INA, LPF, notch\n");

  const timer_config_t timer_cfg = {
    .callback = chip_tick,
    .user_data = s,
  };
  timer_t timer = timer_init(&timer_cfg);
  timer_start(timer, SAMPLE_PERIOD_US, true);
}
