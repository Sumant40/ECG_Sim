/*
 * ECG Electrode + Analog Front-End (AFE) Custom Chip
 * ====================================================
 * Simulates the complete analog signal chain:
 *
 *   Dry skin electrode
 *        ↓
 *   Skin-electrode interface model (impedance, sweat, contact)
 *        ↓
 *   HPF 0.5 Hz (removes baseline wander)
 *        ↓
 *   Instrumentation Amplifier (INA gain, controllable 1–200x)
 *        ↓
 *   LPF 40 Hz (removes EMG + anti-aliases)
 *        ↓
 *   50 Hz Twin-T Notch (removes power-line interference)
 *        ↓
 *   OUT pin: 0–3.3V analog voltage → ESP32 GPIO34
 *
 * Controls in diagram.json:
 *   heartRate   — BPM 40–180
 *   wanderAmp   — baseline wander amplitude in µV
 *   noiseAmp    — EMG noise amplitude in µV
 *   plAmp       — 50 Hz power-line amplitude in µV
 *   inaGain     — INA gain factor (real ADS1292R: up to 12x PGA)
 *
 * HR_IN and WANDER_IN pins are analog inputs:
 *   HR_IN    — if wired to a potentiometer, overrides heartRate control
 *   WANDER_IN — if wired to a potentiometer, overrides wanderAmp control
 *   LEADOFF_IN — digital: HIGH = lead off (flat output at mid-rail)
 */

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define SAMPLE_RATE    500.0f
#define TWO_PI         6.28318530718f
#define VCC            3.3f
#define MID_RAIL       1.65f    /* 0 µV maps to 1.65V = mid-rail */

/* ── Biquad IIR filter (Direct Form II Transposed) ─────────────────────── */
typedef struct {
  float b0, b1, b2, a1, a2;
  float w1, w2;
} Biquad;

static float biquad_process(Biquad *f, float x) {
  float y = f->b0 * x + f->w1;
  f->w1   = f->b1 * x - f->a1 * y + f->w2;
  f->w2   = f->b2 * x - f->a2 * y;
  return y;
}

/* ── Chip state ─────────────────────────────────────────────────────────── */
typedef struct {
  /* pins */
  pin_t pin_out;
  pin_t pin_hr_in;
  pin_t pin_wander_in;
  pin_t pin_leadoff;

  /* controls / attributes */
  uint32_t attr_hr;
  uint32_t attr_wander;
  uint32_t attr_noise;
  uint32_t attr_pl;
  uint32_t attr_gain;

  /* ECG phase accumulator */
  float phase;

  /* noise generators */
  uint32_t lfsr;            /* LFSR for white noise */
  float    wander_phase;    /* 0.25 Hz respiratory wander */
  float    pl_phase;        /* 50 Hz power-line */

  /* filter stages */
  Biquad hpf;       /* Stage 1: HPF 0.5 Hz */
  Biquad lpf1;      /* Stage 3: LPF 40 Hz section 1 */
  Biquad lpf2;      /* Stage 3: LPF 40 Hz section 2 */
  Biquad notch;     /* Stage 4: 50 Hz notch */

} chip_state_t;

/* ── Gaussian pulse ─────────────────────────────────────────────────────── */
static float gaussian(float p, float center, float sigma, float amp) {
  float d = p - center;
  return amp * expf(-(d * d) / (2.0f * sigma * sigma));
}

/* ── PQRST waveform at electrode level (µV) ─────────────────────────────── */
/*
 * Physiological amplitudes at the skin surface (Lead I, dry electrode):
 *   P-wave  : 15 µV  (80–100 ms wide)
 *   Q-wave  : -3 µV  (brief dip, ~20 ms)
 *   R-peak  : 10 µV  (40–60 ms wide, dominant peak)
 *   S-wave  : -4 µV  (brief, ~25 ms)
 *   ST      :  0.5 µV (isoelectric, slight elevation = normal)
 *   T-wave  :  3.5 µV (160 ms wide)
 *   U-wave  :  0.8 µV (small, post-T)
 *
 * After INA gain 100x → R-peak = 1000 µV = 1 mV at ADC input
 * After INA gain 6x  → R-peak =   60 µV         (ADS1292R default PGA)
 *
 * Sigma values are in cardiac cycle phase units (0..1).
 * At 75 BPM: 1 cycle = 800 ms.
 *   P-wave 100 ms  → sigma = 100/800 = 0.125 × (correction 0.6) ≈ 0.020
 *   QRS    80  ms  → sigma = 80/800  = 0.100 × 0.6              ≈ 0.009
 *   T-wave 160 ms  → sigma = 160/800 = 0.200 × 0.6              ≈ 0.038
 */
static float generate_ecg_uV(float p) {
  float P  =  gaussian(p, 0.14f, 0.020f,  15.0f);
  float Q  = -gaussian(p, 0.42f, 0.008f,   3.0f);
  float R  =  gaussian(p, 0.45f, 0.009f,  10.0f);
  float S  = -gaussian(p, 0.48f, 0.009f,   4.0f);
  float ST =  gaussian(p, 0.56f, 0.030f,   0.5f);
  float T  =  gaussian(p, 0.67f, 0.038f,   3.5f);
  float U  =  gaussian(p, 0.80f, 0.018f,   0.8f);
  return P + Q + R + S + ST + T + U;
}

/* ── White noise (LFSR) ─────────────────────────────────────────────────── */
static float white_noise(chip_state_t *s, float amp_uV) {
  s->lfsr ^= s->lfsr << 13;
  s->lfsr ^= s->lfsr >> 17;
  s->lfsr ^= s->lfsr <<  5;
  return ((float)(s->lfsr & 0xFFFF) / 32768.0f - 1.0f) * amp_uV;
}

/* ── Timer callback: fires at 500 Hz ────────────────────────────────────── */
static void chip_timer_cb(void *user_data) {
  chip_state_t *s = (chip_state_t *)user_data;

  /* ── Read controls ─────────────────────────────────────────────────────── */
  /* If HR_IN pin has a potentiometer: use that (0–3.3V → 40–180 BPM) */
  float hr_pin_v = pin_adc_read(s->pin_hr_in);
  float bpm;
  if (hr_pin_v > 0.01f) {
    bpm = 40.0f + (hr_pin_v / VCC) * 140.0f;
  } else {
    bpm = (float)attr_read(s->attr_hr);
  }

  float wander_pin_v = pin_adc_read(s->pin_wander_in);
  float wander_amp;
  if (wander_pin_v > 0.01f) {
    wander_amp = (wander_pin_v / VCC) * 200.0f;
  } else {
    wander_amp = (float)attr_read(s->attr_wander);
  }

  float noise_amp = (float)attr_read(s->attr_noise);
  float pl_amp    = (float)attr_read(s->attr_pl);
  float gain      = (float)attr_read(s->attr_gain);
  if (gain < 1.0f) gain = 1.0f;

  /* ── Lead-off detection ────────────────────────────────────────────────── */
  /* LEADOFF_IN pin: LOW = electrodes on, HIGH = lead off */
  uint32_t leadoff = pin_read(s->pin_leadoff);

  if (leadoff) {
    /* Output mid-rail noise (open circuit artifact) */
    float noise = white_noise(s, 50.0f);
    float v = MID_RAIL + (noise * gain / 1e6f);
    v = fmaxf(0.0f, fminf(VCC, v));
    pin_dac_write(s->pin_out, v);
    return;
  }

  /* ── Phase accumulation (PQRST generator) ──────────────────────────────── */
  float phase_inc = bpm / (60.0f * SAMPLE_RATE);
  s->phase += phase_inc;
  if (s->phase >= 1.0f) s->phase -= 1.0f;

  /* ── Stage 0: Raw electrode signal (µV) ────────────────────────────────── */
  float ecg_uV = generate_ecg_uV(s->phase);

  /* ── Noise components (physically realistic) ────────────────────────────── */
  /* 50 Hz power-line interference */
  s->pl_phase += TWO_PI * 50.0f / SAMPLE_RATE;
  if (s->pl_phase > TWO_PI) s->pl_phase -= TWO_PI;
  float pl_noise = sinf(s->pl_phase) * pl_amp;

  /* Respiratory baseline wander at 0.25 Hz */
  s->wander_phase += TWO_PI * 0.25f / SAMPLE_RATE;
  if (s->wander_phase > TWO_PI) s->wander_phase -= TWO_PI;
  float wander = sinf(s->wander_phase) * wander_amp;

  /* EMG white noise */
  float emg = white_noise(s, noise_amp);

  /* Combined dirty signal at electrode */
  float dirty_uV = ecg_uV + pl_noise + wander + emg;

  /* ── Stage 1: HPF 0.5 Hz — removes baseline wander ────────────────────── */
  /*
   * Butterworth 2nd order HPF, fc=0.5 Hz, fs=500 Hz
   * scipy.signal.butter(2, 0.5/250, btype='high')
   * b = [0.99875078, -1.99750156, 0.99875078]
   * a = [1.0, -1.99750622, 0.99749690]
   */
  float hpf_out = biquad_process(&s->hpf, dirty_uV);

  /* ── Stage 2: INA gain ─────────────────────────────────────────────────── */
  /*
   * ADS1292R PGA: gain register bits PGA_GAIN[2:0]
   *   000 = 6x (default)
   *   001 = 1x
   *   010 = 2x
   *   011 = 3x
   *   100 = 4x
   *   101 = 8x
   *   110 = 12x
   * Set inaGain control to 6 for ADS1292R default.
   * Set to 100 to match MIT-BIH database scale (R-peak ≈ 1000 µV on plotter).
   */
  float amp_uV = hpf_out * gain;

  /* ── Stage 3: LPF 40 Hz — two cascaded biquads ─────────────────────────── */
  /*
   * Butterworth 4th order LPF, fc=40 Hz, fs=500 Hz
   * scipy.signal.butter(4, 40/250, btype='low', output='sos')
   * Section 1:
   *   b = [0.00048845, 0.00097690, 0.00048845]
   *   a = [1.0, -1.76004837, 0.77296192]
   * Section 2:
   *   b = [1.0, 2.0, 1.0]
   *   a = [1.0, -1.90456724, 0.91487003]
   */
  float lpf_out = biquad_process(&s->lpf2,
                    biquad_process(&s->lpf1, amp_uV));

  /* ── Stage 4: 50 Hz notch — IIR Twin-T, Q=30 ───────────────────────────── */
  /*
   * scipy.signal.iirnotch(50, 30, 500)
   * b = [0.96940026, -1.56258775, 0.96940026]
   * a = [1.0, -1.56258775, 0.93880051]
   */
  float filt_uV = biquad_process(&s->notch, lpf_out);

  /* ── Convert filtered µV → 0–3.3V for the analog output pin ────────────── */
  /*
   * Mapping:
   *   0 µV → 1.65V (mid-rail, resting isoelectric line)
   *   +1000 µV (R-peak at gain=100) → ~1.65 + (1000/1e6)*3.3*scale
   *
   * We use a scale factor so R-peak at gain=100 fills ~30% of the ADC range.
   * Scale = 1500 (empirical for gain=100, R-peak 10µV electrode → 1000µV post-amp)
   * → voltage swing = 1000µV / 1,000,000 * 1500 = 0.0015 * 1500 / VCC ≈ 0.68V
   *   which maps to ADC codes ~0–4095 in a realistic range on the ESP32.
   *
   * For gain=6 (ADS1292R default), R-peak = 60 µV post-amp → smaller swing.
   * User can adjust inaGain to see the effect on ADC range.
   */
  float scale = 1500.0f;
  float v_out = MID_RAIL + (filt_uV / 1e6f) * scale;
  v_out = fmaxf(0.0f, fminf(VCC, v_out));

  pin_dac_write(s->pin_out, v_out);
}

/* ── chip_init ──────────────────────────────────────────────────────────── */
void chip_init(void) {
  chip_state_t *s = (chip_state_t *)malloc(sizeof(chip_state_t));
  memset(s, 0, sizeof(chip_state_t));

  /* Pins */
  s->pin_out      = pin_init("OUT",        ANALOG);
  s->pin_hr_in    = pin_init("HR_IN",      ANALOG);
  s->pin_wander_in= pin_init("WANDER_IN",  ANALOG);
  s->pin_leadoff  = pin_init("LEADOFF_IN", INPUT_PULLDOWN);

  /* Attribute controls */
  s->attr_hr     = attr_init("heartRate",  72);
  s->attr_wander = attr_init("wanderAmp",  50);
  s->attr_noise  = attr_init("noiseAmp",   5);
  s->attr_pl     = attr_init("plAmp",      50);
  s->attr_gain   = attr_init("inaGain",    100);

  /* Initialise state */
  s->phase        = 0.0f;
  s->lfsr         = 0xACE1u;
  s->wander_phase = 0.0f;
  s->pl_phase     = 0.0f;

  /* ── Filter coefficients ─────────────────────────────────────────────── */
  /* Stage 1: HPF 2nd order Butterworth, fc=0.5 Hz @ 500 Hz */
  s->hpf.b0 =  0.99875078f; s->hpf.b1 = -1.99750156f; s->hpf.b2 = 0.99875078f;
  s->hpf.a1 = -1.99750622f; s->hpf.a2 =  0.99749690f;
  s->hpf.w1 = s->hpf.w2 = 0.0f;

  /* Stage 3: LPF section 1 (4th order Butterworth fc=40 Hz @ 500 Hz, sos[0]) */
  s->lpf1.b0 = 0.00048845f; s->lpf1.b1 = 0.00097690f; s->lpf1.b2 = 0.00048845f;
  s->lpf1.a1 =-1.76004837f; s->lpf1.a2 = 0.77296192f;
  s->lpf1.w1 = s->lpf1.w2 = 0.0f;

  /* Stage 3: LPF section 2 (sos[1]) */
  s->lpf2.b0 = 1.00000000f; s->lpf2.b1 = 2.00000000f; s->lpf2.b2 = 1.00000000f;
  s->lpf2.a1 =-1.90456724f; s->lpf2.a2 = 0.91487003f;
  s->lpf2.w1 = s->lpf2.w2 = 0.0f;

  /* Stage 4: 50 Hz notch, Q=30 @ 500 Hz */
  s->notch.b0 = 0.96940026f; s->notch.b1 =-1.56258775f; s->notch.b2 = 0.96940026f;
  s->notch.a1 =-1.56258775f; s->notch.a2 = 0.93880051f;
  s->notch.w1 = s->notch.w2 = 0.0f;

  printf("ECG AFE chip init — PQRST + HPF + INA + LPF + Notch ready\n");

  /* 500 Hz timer — fires every 2000 µs */
  const timer_config_t cfg = {
    .callback  = chip_timer_cb,
    .user_data = s,
  };
  timer_t tmr = timer_init(&cfg);
  timer_start(tmr, 2000, true);   /* 2000 µs = 500 Hz */
}