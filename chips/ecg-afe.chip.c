/*
 * ================================================================
 * ECG ELECTRODE + ANALOG FRONT-END CUSTOM CHIP
 * ================================================================
 *
 * Signal chain:
 *
 *   Synthetic ECG
 *        |
 *   Electrode model
 *        |
 *   Baseline / noise
 *        |
 *   HPF 0.5 Hz
 *        |
 *   Instrumentation amplifier (gain up to 900)
 *        |
 *   LPF 40 Hz
 *        |
 *   50 Hz notch
 *        |
 *   OUT  -> ESP32 GPIO34
 *
 *   R-wave detector
 *        |
 *   BEAT -> ESP32 GPIO26
 *
 * ================================================================
 *
 * BUG FIXES (vs original):
 *
 *   1. LEAD-OFF LOGIC INVERTED (critical flatline cause)
 *      LEADOFF pin is INPUT_PULLUP.
 *      Button NOT pressed -> pin HIGH (1) -> leads ARE connected -> run ECG.
 *      Button pressed     -> pin LOW  (0) -> lead off           -> output mid-rail.
 *      Original code tested `if (lead_off)` which is `if (HIGH)` = always true,
 *      so it always output MID_RAIL and returned early. Fixed to `if (!lead_off)`.
 *
 *   2. GAIN CLAMP TOO LOW (secondary flatline cause)
 *      Original clamp was fminf(200.0f, gain). The diagram sets inaGain=450.
 *      With gain=200 and ECG peak ~8 µV, amplified = 1600 µV.
 *      OUTPUT_SCALE = 180, divide by 1e6 -> 0.000288 V swing. Invisible.
 *      Fix: clamp raised to fminf(900.0f, gain) matching the slider max.
 *
 *   3. OUTPUT_SCALE CORRECTED
 *      Full signal-chain analysis (verified by frequency-response math):
 *        R-peak amplitude  = 40 µV (generate_ecg_uV)
 *        After INA gain 450, contact 98%: 40 × 0.98 × 450 = 17 640 µV
 *        Combined LPF+notch gain at ECG freqs ≈ 1.0
 *        filtered ≈ 17 640 µV
 *      Want ±0.30 V DAC swing around MID_RAIL (safe, no clipping):
 *        OUTPUT_SCALE = 0.30 / (17640 / 1e6) = 17.0
 *      This gives ≈ 300 mV R-peak on the Python plotter after ADC conversion.
 *      Previous value 38000 was ×2235 too large → permanent DAC rail clamp.
 *
 * ================================================================
 */

#include "wokwi-api.h"

#include <stdint.h>    /* uint8_t, uint16_t, uint32_t */
#include <stdbool.h>   /* bool, true, false            */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


// ================================================================
// CONSTANTS
// ================================================================

#define SAMPLE_RATE     500.0f
#define SAMPLE_PERIOD   2000   /* microseconds, 500 Hz */

#define TWO_PI          6.28318530718f

#define VCC             3.3f
#define MID_RAIL        1.65f


// ================================================================
// BIQUAD FILTER — Direct Form II Transposed
// ================================================================

typedef struct
{
    float b0, b1, b2;
    float a1, a2;
    float w1, w2;
} Biquad;


static float biquad_process(Biquad *f, float x)
{
    float y   = f->b0 * x + f->w1;
    f->w1     = f->b1 * x - f->a1 * y + f->w2;
    f->w2     = f->b2 * x - f->a2 * y;
    return y;
}

static void biquad_reset(Biquad *f)
{
    f->w1 = 0.0f;
    f->w2 = 0.0f;
}


// ================================================================
// CHIP STATE
// ================================================================

typedef struct
{
    /* Pins */
    pin_t pin_out;
    pin_t pin_hr_in;
    pin_t pin_leadoff;
    pin_t pin_beat;

    /* Wokwi controls */
    uint32_t attr_hr;
    uint32_t attr_wander;
    uint32_t attr_noise;
    uint32_t attr_mains;
    uint32_t attr_gain;
    uint32_t attr_contact;
    uint32_t attr_cmrr;

    /* ECG state */
    float phase;

    /* Noise state */
    uint32_t lfsr;
    float    wander_phase;
    float    mains_phase;

    /* Filters */
    Biquad hpf;
    Biquad lpf1;
    Biquad lpf2;
    Biquad notch;

    /* Beat detector */
    bool  beat_state;
    int   beat_refractory;

} chip_state_t;


// ================================================================
// GAUSSIAN HELPER
// ================================================================

static float gaussian(float p, float center, float sigma, float amplitude)
{
    float d = p - center;
    return amplitude * expf(-(d * d) / (2.0f * sigma * sigma));
}


// ================================================================
// SYNTHETIC ECG IN MICROVOLTS
//
// One cardiac cycle: phase 0..1
// Amplitudes chosen so that after INA gain 450 and OUTPUT_SCALE
// the signal fills roughly half the ADC range around MID_RAIL.
// ================================================================

static float generate_ecg_uV(float p)
{
    float P  = gaussian(p, 0.18f,  0.030f,  10.0f);   /* atrial depol   */
    float Q  = -gaussian(p, 0.405f, 0.010f,   2.5f);  /* Q dip          */
    float R  = gaussian(p, 0.430f,  0.013f,  40.0f);  /* R peak (bigger)*/
    float S  = -gaussian(p, 0.458f, 0.011f,   5.0f);  /* S dip          */
    float ST = gaussian(p, 0.55f,   0.035f,   0.4f);  /* ST segment     */
    float T  = gaussian(p, 0.68f,   0.055f,   8.0f);  /* T wave         */
    float U  = gaussian(p, 0.82f,   0.022f,   0.6f);  /* U wave (small) */

    return P + Q + R + S + ST + T + U;
}


// ================================================================
// WHITE NOISE — Galois LFSR, 32-bit
// ================================================================

static float white_noise(chip_state_t *s, float amplitude)
{
    s->lfsr ^= s->lfsr << 13;
    s->lfsr ^= s->lfsr >> 17;
    s->lfsr ^= s->lfsr << 5;

    float n = ((float)(s->lfsr & 0xFFFF) / 32768.0f) - 1.0f;
    return n * amplitude;
}


// ================================================================
// BEAT OUTPUT
//
// Triggered from clean ECG phase, not from the noisy ADC.
// ================================================================

static void generate_beat(chip_state_t *s, float phase)
{
    bool r_region = (phase >= 0.412f && phase <= 0.448f);

    if (r_region && !s->beat_state && s->beat_refractory <= 0)
    {
        pin_write(s->pin_beat, 1);
        s->beat_state      = true;
        s->beat_refractory = 80;
    }

    if (!r_region && s->beat_state)
    {
        pin_write(s->pin_beat, 0);
        s->beat_state = false;
    }

    if (s->beat_refractory > 0)
        s->beat_refractory--;
}


// ================================================================
// TIMER CALLBACK — runs at 500 Hz
// ================================================================

static void chip_timer_cb(void *user_data)
{
    chip_state_t *s = (chip_state_t *)user_data;


    // ============================================================
    // READ HEART RATE
    // ============================================================

    float hr_voltage = pin_adc_read(s->pin_hr_in);

    float bpm;
    if (hr_voltage > 0.01f)
        bpm = 40.0f + (hr_voltage / VCC) * 140.0f;
    else
        bpm = (float)attr_read(s->attr_hr);

    bpm = fmaxf(40.0f, fminf(180.0f, bpm));


    // ============================================================
    // READ ARTIFACT CONTROLS
    // ============================================================

    float wander_amp = (float)attr_read(s->attr_wander);
    float noise_amp  = (float)attr_read(s->attr_noise);
    float mains_amp  = (float)attr_read(s->attr_mains);

    float gain = (float)attr_read(s->attr_gain);
    /* FIX #2: raise the clamp ceiling to 900 to match the slider */
    gain = fmaxf(1.0f, fminf(900.0f, gain));

    /* Contact quality (0-100) scales the signal amplitude */
    float contact = (float)attr_read(s->attr_contact) / 100.0f;
    contact = fmaxf(0.0f, fminf(1.0f, contact));

    /* CMRR (60-120 dB) — higher CMRR suppresses common-mode noise */
    float cmrr_db    = (float)attr_read(s->attr_cmrr);
    float cmrr_lin   = powf(10.0f, cmrr_db / 20.0f);   /* voltage ratio */


    // ============================================================
    // FIX #1: LEAD-OFF LOGIC
    //
    // LEADOFF pin = INPUT_PULLUP.
    //   HIGH (1) = button NOT pressed = electrodes connected = normal.
    //   LOW  (0) = button pressed     = lead off             = flatline.
    //
    // Original code: if (lead_off) { output MID_RAIL; return; }
    //   -> lead_off was HIGH at rest -> always flatlined!
    //
    // Fixed:  if (!lead_off) { output MID_RAIL; return; }
    // ============================================================

    uint32_t lead_off = pin_read(s->pin_leadoff);

    if (!lead_off)   /* lead is OFF — button pressed (pin LOW) */
    {
        pin_dac_write(s->pin_out, MID_RAIL);
        pin_write(s->pin_beat, 0);
        s->beat_state = false;

        /* Reset filter states so there's no transient when leads reconnect */
        biquad_reset(&s->hpf);
        biquad_reset(&s->lpf1);
        biquad_reset(&s->lpf2);
        biquad_reset(&s->notch);

        return;
    }


    // ============================================================
    // UPDATE ECG PHASE
    // ============================================================

    float phase_increment = bpm / (60.0f * SAMPLE_RATE);
    s->phase += phase_increment;
    if (s->phase >= 1.0f)
        s->phase -= 1.0f;


    // ============================================================
    // BEAT OUTPUT
    // ============================================================

    generate_beat(s, s->phase);


    // ============================================================
    // ECG MORPHOLOGY (microvolts, scaled by contact quality)
    // ============================================================

    float ecg = generate_ecg_uV(s->phase) * contact;


    // ============================================================
    // BASELINE WANDER (respiration artifact ~0.25 Hz)
    // ============================================================

    s->wander_phase += TWO_PI * 0.25f / SAMPLE_RATE;
    if (s->wander_phase >= TWO_PI)
        s->wander_phase -= TWO_PI;

    float wander = sinf(s->wander_phase) * wander_amp;


    // ============================================================
    // 50 Hz MAINS INTERFERENCE
    // Attenuated by CMRR: actual common-mode leakage = amp / cmrr_lin
    // ============================================================

    s->mains_phase += TWO_PI * 50.0f / SAMPLE_RATE;
    if (s->mains_phase >= TWO_PI)
        s->mains_phase -= TWO_PI;

    float mains = sinf(s->mains_phase) * (mains_amp / fmaxf(1.0f, cmrr_lin * 0.01f));


    // ============================================================
    // EMG NOISE
    // ============================================================

    float noise = white_noise(s, noise_amp);


    // ============================================================
    // COMPOSITE ELECTRODE SIGNAL
    // ============================================================

    float electrode_signal = ecg + wander + mains + noise;


    // ============================================================
    // HPF 0.5 Hz  — removes DC / electrode offset
    // ============================================================

    float hpf_out = biquad_process(&s->hpf, electrode_signal);


    // ============================================================
    // INSTRUMENTATION AMPLIFIER
    // ============================================================

    float amplified = hpf_out * gain;


    // ============================================================
    // LPF 40 Hz (4th-order Butterworth, two biquad sections)
    // ============================================================

    float lpf_out = biquad_process(&s->lpf2,
                        biquad_process(&s->lpf1, amplified));


    // ============================================================
    // 50 Hz NOTCH
    // ============================================================

    float filtered = biquad_process(&s->notch, lpf_out);


    // ============================================================
    // CLIP — prevent filter transients from hitting the rail
    // ============================================================

    filtered = fmaxf(-25000.0f, fminf(25000.0f, filtered));  /* ±0.43V max DAC swing */


    // ============================================================
    // OUTPUT SCALE
    //
    // Signal budget (gain=450, contact=98%):
    //   R-peak = 40 µV × 0.98 × 450 = 17 640 µV
    //   LPF + notch gain at ECG frequencies ≈ 1.0 (freq-response verified)
    //   filtered ≈ 17 640 µV
    //
    // Want ±0.30 V DAC swing around MID_RAIL:
    //   OUTPUT_SCALE = 0.30 / (17640 / 1e6) = 17.0
    //
    // Plotter sees R-peak as ≈ 300 mV after ADC conversion.
    // DAC output stays in 1.35–1.95 V range — well within 0–3.3 V rail.
    // ============================================================

    const float OUTPUT_SCALE = 17.0f;

    float output_voltage = MID_RAIL + (filtered / 1000000.0f) * OUTPUT_SCALE;


    // ============================================================
    // DAC SAFETY CLAMP (stay inside 3.3 V rail with margin)
    // ============================================================

    output_voltage = fmaxf(0.10f, fminf(3.20f, output_voltage));


    // ============================================================
    // WRITE TO DAC
    // ============================================================

    pin_dac_write(s->pin_out, output_voltage);
}


// ================================================================
// CHIP INITIALIZATION
// ================================================================

void chip_init(void)
{
    chip_state_t *s = (chip_state_t *)malloc(sizeof(chip_state_t));
    memset(s, 0, sizeof(chip_state_t));


    // ============================================================
    // PINS
    // ============================================================

    s->pin_out     = pin_init("OUT",     ANALOG);
    s->pin_hr_in   = pin_init("HR_IN",   ANALOG);
    s->pin_leadoff = pin_init("LEADOFF", INPUT_PULLUP);
    s->pin_beat    = pin_init("BEAT",    OUTPUT);


    // ============================================================
    // WOKWI CONTROLS
    // ============================================================

    s->attr_hr      = attr_init("heartRate",      72);
    s->attr_wander  = attr_init("wanderAmp",       0);
    s->attr_noise   = attr_init("noiseAmp",        0);
    s->attr_mains   = attr_init("plAmp",           0);
    s->attr_gain    = attr_init("inaGain",       450);
    s->attr_contact = attr_init("contactQuality", 98);
    s->attr_cmrr    = attr_init("cmrrDb",        100);


    // ============================================================
    // INITIAL STATE
    // ============================================================

    s->phase         = 0.0f;
    s->lfsr          = 0xACE1u;
    s->wander_phase  = 0.0f;
    s->mains_phase   = 0.0f;
    s->beat_state    = false;
    s->beat_refractory = 0;


    // ============================================================
    // HPF 0.5 Hz  (Butterworth, Fs=500 Hz)
    // ============================================================

    s->hpf.b0 =  0.99556697f;
    s->hpf.b1 = -1.99113394f;
    s->hpf.b2 =  0.99556697f;
    s->hpf.a1 = -1.99111429f;
    s->hpf.a2 =  0.99115360f;


    // ============================================================
    // 4th-order Butterworth LPF, 40 Hz, Fs=500 Hz
    // Section 1
    // ============================================================

    s->lpf1.b0 =  0.00223489f;
    s->lpf1.b1 =  0.00446978f;
    s->lpf1.b2 =  0.00223489f;
    s->lpf1.a1 = -1.21281209f;
    s->lpf1.a2 =  0.38400416f;

    /* Section 2 */
    s->lpf2.b0 =  1.0f;
    s->lpf2.b1 =  2.0f;
    s->lpf2.b2 =  1.0f;
    s->lpf2.a1 = -1.47979889f;
    s->lpf2.a2 =  0.68867695f;


    // ============================================================
    // 50 Hz NOTCH (Fs=500 Hz)
    // ============================================================

    s->notch.b0 =  0.99110364f;
    s->notch.b1 = -1.60363937f;
    s->notch.b2 =  0.99110364f;
    s->notch.a1 = -1.60363937f;
    s->notch.a2 =  0.98220727f;


    // ============================================================
    // STARTUP MESSAGE
    // ============================================================

    printf("[ECG-AFE] v2 — PQRSTU synthetic ECG ready\n");
    printf("[ECG-AFE] OUT  -> GPIO34  |  BEAT -> GPIO26\n");
    printf("[ECG-AFE] Default HR = 72 BPM | INA Gain = 450\n");
    printf("[ECG-AFE] Lead-off: button pressed = LOW = flatline\n");
    printf("[ECG-AFE] Fixes: lead-off polarity, gain clamp, output scale\n");


    // ============================================================
    // 500 Hz TIMER
    // ============================================================

    const timer_config_t config =
    {
        .callback  = chip_timer_cb,
        .user_data = s
    };

    timer_t timer = timer_init(&config);
    timer_start(timer, SAMPLE_PERIOD, true);
}