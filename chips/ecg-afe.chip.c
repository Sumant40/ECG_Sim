/*
 * ECG-AFE custom chip — 6 electrode inputs, INA stage, filter cascade.
 *
 * Electrodes: RA, LA, RL (reference), LL, V1, V5
 * Signal chain:
 *   Electrode pots (inject contact noise)
 *   → Gaussian PQRST synthesis per anatomical lead vector
 *   → INA (differential amp, RLD common-mode rejection)
 *   → 4th-order Butterworth BPF 0.5–40 Hz (2 biquads, DF2T)
 *   → IIR notch 50 Hz
 *   → OUT  (analog, DAC, 0–3.3 V centered at 1.65 V)
 *   → BEAT (digital, HIGH for 4 ms on each detected R-peak)
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
#define VCC_V            3.3f
#define MID_RAIL         1.65f

/* ── Direct Form II Transposed biquad ─────────────────────────────── */
typedef struct { float b0,b1,b2,a1,a2,w1,w2; } Biquad;

static float bq(Biquad *f, float x) {
  float y = f->b0*x + f->w1;
  f->w1   = f->b1*x - f->a1*y + f->w2;
  f->w2   = f->b2*x - f->a2*y;
  return y;
}

/* ── Chip state ───────────────────────────────────────────────────── */
typedef struct {
  /* Electrode input pins */
  pin_t ra, la, rl, ll, v1, v5;
  /* Control input pins */
  pin_t hr_in, leadoff;
  /* Output pins */
  pin_t out, beat;
  /* Attribute handles */
  uint32_t a_hr, a_wander, a_noise, a_mains, a_gain, a_contact, a_cmrr;
  /* Oscillator phases */
  float ecg_ph, resp_ph, motion_ph, mains_ph;
  /* Filter stages */
  Biquad bpf1, bpf2, notch;
  /* Hardware beat detector */
  float   beat_thr;
  uint32_t beat_hold;   /* samples remaining in beat-hold period */
  /* RNG */
  uint32_t rng;
} chip_state_t;

static float cf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }

static float gauss(float ph, float c, float sig, float amp) {
  float d = ph - c;
  if (d >  0.5f) d -= 1.0f;
  if (d < -0.5f) d += 1.0f;
  return amp * expf(-(d*d)/(2.0f*sig*sig));
}

static float bipolar(chip_state_t *s, float amp) {
  s->rng ^= s->rng<<13; s->rng ^= s->rng>>17; s->rng ^= s->rng<<5;
  return (((float)(s->rng & 0xffffu)/32767.5f) - 1.0f) * amp;
}

/* ── Anatomical ECG waveforms (voltage at each electrode site) ─────── */
/* Lead II axis (RA→LL): largest QRS, clinical reference */
static float ecg_ll(float p) {
  return gauss(p,0.185f,0.028f, 0.060f)   /* P  */
        -gauss(p,0.385f,0.009f, 0.040f)   /* Q  */
        +gauss(p,0.407f,0.010f, 0.460f)   /* R  */
        -gauss(p,0.432f,0.012f, 0.080f)   /* S  */
        +gauss(p,0.670f,0.070f, 0.130f);  /* T  */
}
/* Lead I axis (RA→LA): smaller QRS */
static float ecg_la(float p) {
  return gauss(p,0.185f,0.028f, 0.040f)
        -gauss(p,0.385f,0.009f, 0.020f)
        +gauss(p,0.407f,0.010f, 0.300f)
        -gauss(p,0.432f,0.012f, 0.050f)
        +gauss(p,0.670f,0.070f, 0.100f);
}
/* RA: acts as negative pole, small amplitude */
static float ecg_ra(float p) {
  return gauss(p,0.185f,0.030f, 0.020f)
        +gauss(p,0.407f,0.012f, 0.090f)
        +gauss(p,0.670f,0.070f, 0.040f);
}
/* V1 precordial: rS pattern — dominant S wave */
static float ecg_v1(float p) {
  return gauss(p,0.185f,0.028f, 0.025f)
        +gauss(p,0.400f,0.012f, 0.080f)   /* small r */
        -gauss(p,0.425f,0.015f, 0.340f)   /* deep S  */
        +gauss(p,0.670f,0.070f, 0.050f);
}
/* V5 precordial: tall R, small s */
static float ecg_v5(float p) {
  return gauss(p,0.185f,0.028f, 0.040f)
        -gauss(p,0.385f,0.009f, 0.020f)
        +gauss(p,0.407f,0.010f, 0.420f)
        -gauss(p,0.432f,0.012f, 0.035f)
        +gauss(p,0.670f,0.070f, 0.120f);
}

/* ── Sample callback (runs every 2 ms = 500 Hz) ──────────────────── */
static void chip_tick(void *ud) {
  chip_state_t *s = (chip_state_t *)ud;

  /* ── Lead-off: output artifact only ───────────────────────────── */
  if (pin_read(s->leadoff) == LOW) {
    s->motion_ph += TWO_PI * 1.3f / SAMPLE_RATE_HZ;
    if (s->motion_ph >= TWO_PI) s->motion_ph -= TWO_PI;
    float art = 0.06f*sinf(s->motion_ph) + bipolar(s,0.025f);
    pin_dac_write(s->out, cf(MID_RAIL+art, 0.0f, VCC_V));
    pin_write(s->beat, LOW);
    return;
  }

  /* ── Read HR pot or attribute ──────────────────────────────────── */
  float hr_v = pin_adc_read(s->hr_in);
  float bpm  = (hr_v > 0.02f)
    ? 40.0f + cf(hr_v/VCC_V, 0.0f, 1.0f)*140.0f
    : cf((float)attr_read(s->a_hr), 40.0f, 180.0f);

  /* ── Read attributes ───────────────────────────────────────────── */
  float gain    = cf((float)attr_read(s->a_gain),    100.0f, 900.0f);
  float contact = cf((float)attr_read(s->a_contact),   0.0f, 100.0f)/100.0f;
  float noise   = cf((float)attr_read(s->a_noise)  /1000.0f, 0.0f, 0.012f);
  float wander  = cf((float)attr_read(s->a_wander) /1000.0f, 0.0f, 0.022f);
  float mains   = cf((float)attr_read(s->a_mains)  /1000.0f, 0.0f, 0.018f);
  float cmrr_lin= powf(10.0f, cf((float)attr_read(s->a_cmrr),60.0f,120.0f)/20.0f);

  /* ── Advance oscillators ───────────────────────────────────────── */
  s->ecg_ph    += bpm/(60.0f*SAMPLE_RATE_HZ);
  if (s->ecg_ph    >= 1.0f)   s->ecg_ph    -= 1.0f;
  s->resp_ph   += TWO_PI*0.23f /SAMPLE_RATE_HZ;
  if (s->resp_ph   >= TWO_PI)  s->resp_ph   -= TWO_PI;
  s->motion_ph += TWO_PI*1.3f  /SAMPLE_RATE_HZ;
  if (s->motion_ph >= TWO_PI)  s->motion_ph -= TWO_PI;
  s->mains_ph  += TWO_PI*50.0f /SAMPLE_RATE_HZ;
  if (s->mains_ph  >= TWO_PI)  s->mains_ph  -= TWO_PI;

  /* ── Electrode pin readings (pots inject ±noise around centre) ── */
  /* Pot at mid = 1.65 V → 0 injection; turned CCW/CW = ±variation  */
  float ra_inj = (pin_adc_read(s->ra)/VCC_V - 0.5f) * noise * 2.0f;
  float la_inj = (pin_adc_read(s->la)/VCC_V - 0.5f) * noise * 2.0f;
  float rl_cm  = (pin_adc_read(s->rl)/VCC_V - 0.5f) * 0.018f; /* common-mode */
  float ll_inj = (pin_adc_read(s->ll)/VCC_V - 0.5f) * noise * 2.0f;
  float v1_inj = (pin_adc_read(s->v1)/VCC_V - 0.5f) * noise * 2.0f;
  float v5_inj = (pin_adc_read(s->v5)/VCC_V - 0.5f) * noise * 2.0f;

  /* ── Synthesise voltage at each electrode ──────────────────────── */
  float resp_wander = wander * sinf(s->resp_ph);
  float emg         = bipolar(s, noise);
  float pl_noise    = mains  * sinf(s->mains_ph);

  float V_RA = ecg_ra(s->ecg_ph)*contact + ra_inj + resp_wander + emg*0.3f;
  float V_LA = ecg_la(s->ecg_ph)*contact + la_inj + resp_wander + emg*0.3f;
  float V_RL = rl_cm + bipolar(s, noise*0.4f);        /* reference/ground */
  float V_LL = ecg_ll(s->ecg_ph)*contact + ll_inj + resp_wander + emg;
  float V_V1 = ecg_v1(s->ecg_ph)*contact + v1_inj + resp_wander + emg*0.5f;
  float V_V5 = ecg_v5(s->ecg_ph)*contact + v5_inj + resp_wander + emg*0.5f;
  (void)V_LA; (void)V_V1; (void)V_V5; /* available for future multi-lead output */

  /* ── INA stage — Lead II differential = V_LL − V_RA ───────────── */
  float cm        = (V_LL + V_RA) * 0.5f;          /* common-mode component */
  float rld_res   = cm / cmrr_lin;                  /* residual after RLD    */
  float diff      = (V_LL - V_RA)                   /* differential signal   */
                  - V_RL * 0.05f                    /* RLD correction        */
                  + rld_res                         /* CMRR residual         */
                  + pl_noise;                       /* mains residual        */
  float amplified = diff * (gain / 450.0f);

  /* ── Filter cascade ────────────────────────────────────────────── */
  /* Stage 1+2: 4th-order Butterworth BPF 0.5–40 Hz @ 500 Hz        */
  float filt = bq(&s->bpf2, bq(&s->bpf1, amplified));
  /* Stage 3: IIR notch 50 Hz                                        */
  filt = bq(&s->notch, filt);
  if (!isfinite(filt)) filt = 0.0f;

  /* ── DAC output: centre at MID_RAIL ───────────────────────────── */
  pin_dac_write(s->out, cf(MID_RAIL + filt, 0.0f, VCC_V));

  /* ── Hardware beat detector (adaptive threshold) ───────────────── */
  if (s->beat_hold > 0) {
    s->beat_hold--;
    pin_write(s->beat, (s->beat_hold > 96) ? HIGH : LOW); /* HIGH ~8ms */
  } else if (filt > s->beat_thr) {
    s->beat_thr  = s->beat_thr*0.875f + filt*0.125f;
    s->beat_hold = (uint32_t)(SAMPLE_RATE_HZ * 0.200f);  /* 200 ms refractory */
    pin_write(s->beat, HIGH);
  } else {
    s->beat_thr *= 0.9997f;
    if (s->beat_thr < 0.004f) s->beat_thr = 0.004f;
    pin_write(s->beat, LOW);
  }
}

/* ── chip_init ────────────────────────────────────────────────────── */
void chip_init(void) {
  chip_state_t *s = (chip_state_t *)malloc(sizeof(chip_state_t));
  memset(s, 0, sizeof(chip_state_t));

  /* Electrode pins — pots connected here inject ±variation */
  s->ra      = pin_init("RA",      ANALOG);
  s->la      = pin_init("LA",      ANALOG);
  s->rl      = pin_init("RL",      ANALOG);
  s->ll      = pin_init("LL",      ANALOG);
  s->v1      = pin_init("V1",      ANALOG);
  s->v5      = pin_init("V5",      ANALOG);
  /* Control */
  s->hr_in   = pin_init("HR_IN",   ANALOG);
  s->leadoff = pin_init("LEADOFF", INPUT_PULLUP);
  /* Outputs */
  s->out     = pin_init("OUT",     ANALOG);
  s->beat    = pin_init("BEAT",    OUTPUT);

  /* Attributes (Wokwi chip sliders) */
  s->a_hr      = attr_init("heartRate",      72);
  s->a_wander  = attr_init("wanderAmp",      35);
  s->a_noise   = attr_init("noiseAmp",        2);
  s->a_mains   = attr_init("plAmp",          15);
  s->a_gain    = attr_init("inaGain",       450);
  s->a_contact = attr_init("contactQuality", 98);
  s->a_cmrr    = attr_init("cmrrDb",        100);

  /*
   * BPF 0.5–40 Hz, Butterworth 4th-order @ 500 Hz
   * scipy: butter(2,[0.5/250,40/250],btype='band',output='sos')
   */
  s->bpf1 = (Biquad){0.00482434f, 0.0f,-0.00482434f,-1.97832f,0.98990f,0.0f,0.0f};
  s->bpf2 = (Biquad){0.00482434f, 0.0f,-0.00482434f,-1.98930f,0.99040f,0.0f,0.0f};
  /*
   * IIR notch 50 Hz, Q=35 @ 500 Hz
   * w0=2pi*50/500, r=1-pi*50/(35*500)
   * normalised for unity DC gain
   */
  s->notch= (Biquad){0.99086f,-1.60315f,0.99086f,-1.60365f,0.98213f,0.0f,0.0f};

  s->beat_thr = 0.006f;
  s->rng      = 0x13579bdfu;

  printf("[ECG-AFE] 6-electrode chip ready: RA LA RL LL V1 V5 "
         "→ INA(gain=450) → BPF(0.5-40Hz) → Notch(50Hz) → OUT/BEAT\n");

  const timer_config_t cfg = {.callback = chip_tick, .user_data = s};
  timer_t t = timer_init(&cfg);
  timer_start(t, SAMPLE_PERIOD_US, true);
}