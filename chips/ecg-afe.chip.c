/*
 * ================================================================
 * ECG 6-ELECTRODE AFE - STABLE WOKWI VERSION
 * ================================================================
 *
 * Signal path:
 *
 *   Synthetic PQRSTU ECG
 *           |
 *      Electrode model
 *           |
 *       INA gain
 *           |
 *       OUT 1.65 V centered
 *           |
 *      ESP32 GPIO34
 *
 *   R-wave timing -> BEAT -> ESP32 GPIO26
 *
 * Lead-off:
 *   LEADOFF HIGH = normal
 *   LEADOFF LOW  = lead-off
 *
 * IMPORTANT:
 * The synthetic ECG is already smooth and band-limited. The
 * previous custom-chip IIR chain produced ringing and extra
 * peaks. This stable version therefore keeps the AFE signal
 * conditioning simple and leaves visualization filtering to
 * the ESP32/plotter.
 */

#include "wokwi-api.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define SAMPLE_RATE   500.0f
#define SAMPLE_PERIOD 2000
#define TWO_PI         6.28318530718f
#define VCC            3.3f
#define MID_RAIL       1.65f

typedef struct
{
    pin_t ra;
    pin_t la;
    pin_t rl;
    pin_t ll;
    pin_t v1;
    pin_t v5;

    pin_t hr_in;
    pin_t leadoff;

    pin_t out;
    pin_t beat;

    uint32_t attr_hr;
    uint32_t attr_wander;
    uint32_t attr_noise;
    uint32_t attr_mains;
    uint32_t attr_gain;
    uint32_t attr_contact;
    uint32_t attr_cmrr;

    float phase;
    float wander_phase;
    float mains_phase;

    uint32_t lfsr;

    bool beat_state;
    int beat_samples;
} chip_state_t;


/* ------------------------------------------------------------
 * Gaussian pulse
 * ------------------------------------------------------------ */
static float gaussian(
    float phase,
    float center,
    float sigma,
    float amplitude)
{
    float d = phase - center;

    return amplitude *
           expf(
               -(d * d) /
               (2.0f * sigma * sigma)
           );
}


/* ------------------------------------------------------------
 * Clean Lead-II-like PQRSTU waveform
 *
 * Amplitudes are in arbitrary microvolt-like units.
 * Chosen for clear visual morphology:
 *
 *       P          R
 *      /\         /\
 * ____/  \_______/  \___/\____
 *              Q  S      T
 * ------------------------------------------------------------ */
static float ecg_uV(float p)
{
    // --------------------------------------------------------
    // P wave
    // Small, broad atrial depolarization
    // --------------------------------------------------------
    float P =
        gaussian(
            p,
            0.18f,
            0.035f,
            3.0f
        );

    // --------------------------------------------------------
    // Q wave
    // Small negative deflection
    // --------------------------------------------------------
    float Q =
        -gaussian(
            p,
            0.405f,
            0.009f,
            2.0f
        );

    // --------------------------------------------------------
    // R wave
    // Main ventricular depolarization
    // Broadened so it is not a needle spike
    // --------------------------------------------------------
    float R =
        gaussian(
            p,
            0.430f,
            0.020f,
            25.0f
        );

    // --------------------------------------------------------
    // S wave
    // Moderate negative deflection
    // --------------------------------------------------------
    float S =
        -gaussian(
            p,
            0.468f,
            0.014f,
            7.0f
        );

    // --------------------------------------------------------
    // T wave
    // Broad and significantly smaller than R
    // --------------------------------------------------------
    float T =
        gaussian(
            p,
            0.68f,
            0.075f,
            7.0f
        );

    // No visible U wave for this clean presentation

    // Small DC compensation so the cycle is centered near
    // the isoelectric line.
    const float baseline =
        -0.65f;

    return
        P +
        Q +
        R +
        S +
        T +
        baseline;
}

/* ------------------------------------------------------------
 * White noise
 * ------------------------------------------------------------ */
static float white_noise(
    chip_state_t *s,
    float amplitude)
{
    s->lfsr ^= s->lfsr << 13;
    s->lfsr ^= s->lfsr >> 17;
    s->lfsr ^= s->lfsr << 5;

    float n =
        ((float)(s->lfsr & 0xFFFF) /
         32768.0f) - 1.0f;

    return n * amplitude;
}


/* ------------------------------------------------------------
 * R-wave pulse
 *
 * One pulse per cycle, synchronized with R wave.
 * Pulse width = 40 ms.
 * ------------------------------------------------------------ */
static void update_beat(
    chip_state_t *s)
{
    if (s->beat_samples > 0)
    {
        pin_write(
            s->beat,
            1
        );

        s->beat_samples--;

        return;
    }

    pin_write(
        s->beat,
        0
    );
}


/* ------------------------------------------------------------
 * Timer callback: 500 Hz
 * ------------------------------------------------------------ */
static void chip_timer_cb(
    void *user_data)
{
    chip_state_t *s =
        (chip_state_t *)user_data;


    /* --------------------------------------------------------
     * Lead-off
     *
     * INPUT_PULLUP:
     * HIGH = normal
     * LOW  = lead off
     * -------------------------------------------------------- */
    uint32_t lead_off =
        pin_read(
            s->leadoff
        );

    if (!lead_off)
    {
        pin_dac_write(
            s->out,
            MID_RAIL
        );

        pin_write(
            s->beat,
            0
        );

        s->beat_state = false;
        s->beat_samples = 0;

        return;
    }


    /* --------------------------------------------------------
     * Heart rate
     *
     * IMPORTANT:
     * We intentionally use the Wokwi heartRate control rather
     * than the HR potentiometer. The old HR potentiometer
     * override was causing 140-170 BPM in the plot.
     * -------------------------------------------------------- */
    float bpm =
        (float)attr_read(
            s->attr_hr
        );

    bpm =
        fmaxf(
            40.0f,
            fminf(
                180.0f,
                bpm
            )
        );


    /* --------------------------------------------------------
     * Artifact controls
     * -------------------------------------------------------- */
    float wander_amp =
        (float)attr_read(
            s->attr_wander
        );

    float noise_amp =
        (float)attr_read(
            s->attr_noise
        );

    float mains_amp =
        (float)attr_read(
            s->attr_mains
        );


    /* --------------------------------------------------------
     * INA gain
     *
     * Gain affects the visible amplitude.
     * -------------------------------------------------------- */
    float gain =
        (float)attr_read(
            s->attr_gain
        );

    gain =
        fmaxf(
            100.0f,
            fminf(
                900.0f,
                gain
            )
        );


    float contact =
        (float)attr_read(
            s->attr_contact
        ) / 100.0f;

    contact =
        fmaxf(
            0.0f,
            fminf(
                1.0f,
                contact
            )
        );


    /* --------------------------------------------------------
     * Advance cardiac phase
     * -------------------------------------------------------- */
    float old_phase =
        s->phase;

    float phase_inc =
        bpm /
        (
            60.0f *
            SAMPLE_RATE
        );

    s->phase += phase_inc;

    if (s->phase >= 1.0f)
        s->phase -= 1.0f;


    /* --------------------------------------------------------
     * Generate R-wave pulse exactly when phase crosses R.
     * -------------------------------------------------------- */
    if (
        old_phase < 0.430f &&
        s->phase >= 0.430f
    )
    {
        s->beat_samples = 20;  // 20 * 2 ms = 40 ms
        s->beat_state = true;
    }

    update_beat(s);


    /* --------------------------------------------------------
     * ECG
     * -------------------------------------------------------- */
    float signal =
        ecg_uV(
            s->phase
        ) *
        contact;


    /* --------------------------------------------------------
     * Baseline wander
     * -------------------------------------------------------- */
    s->wander_phase +=
        TWO_PI *
        0.25f /
        SAMPLE_RATE;

    if (s->wander_phase >= TWO_PI)
        s->wander_phase -= TWO_PI;

    signal +=
        sinf(s->wander_phase) *
        wander_amp;


    /* --------------------------------------------------------
     * 50 Hz mains artifact
     *
     * Small by default. It is disabled in diagram.json.
     * -------------------------------------------------------- */
    s->mains_phase +=
        TWO_PI *
        50.0f /
        SAMPLE_RATE;

    if (s->mains_phase >= TWO_PI)
        s->mains_phase -= TWO_PI;

    float mains =
        sinf(s->mains_phase) *
        mains_amp;

    signal += mains;


    /* --------------------------------------------------------
     * EMG noise
     * -------------------------------------------------------- */
    signal +=
        white_noise(
            s,
            noise_amp
        );


    /* --------------------------------------------------------
     * Instrumentation amplifier
     *
     * signal is treated as microvolt-like.
     * -------------------------------------------------------- */
    float amplified =
        signal *
        gain;


    /* --------------------------------------------------------
     * Output scaling
     *
     * With:
     *   R = 25 uV
     *   gain = 450
     *   contact = 0.98
     *
     * R at INA ≈ 11025 uV.
     *
     * scale 25 gives:
     *   11025/1e6 * 25 ≈ 0.276 V
     *
     * So the DAC remains around:
     *   1.37 V ... 1.93 V
     *
     * which is ideal for the ESP32 ADC.
     * -------------------------------------------------------- */
    const float OUTPUT_SCALE =
        25.0f;

    float output =
        MID_RAIL +
        (
            amplified /
            1000000.0f
        ) *
        OUTPUT_SCALE;


    /* --------------------------------------------------------
     * DAC safety
     * -------------------------------------------------------- */
    output =
        fmaxf(
            0.20f,
            fminf(
                3.10f,
                output
            )
        );


    pin_dac_write(
        s->out,
        output
    );
}


/* ------------------------------------------------------------
 * Initialize chip
 * ------------------------------------------------------------ */
void chip_init(void)
{
    chip_state_t *s =
        (chip_state_t *)
        malloc(
            sizeof(
                chip_state_t
            )
        );

    if (!s)
        return;

    memset(
        s,
        0,
        sizeof(
            chip_state_t
        )
    );


    /* Electrode pins */
    s->ra =
        pin_init("RA", ANALOG);

    s->la =
        pin_init("LA", ANALOG);

    s->rl =
        pin_init("RL", ANALOG);

    s->ll =
        pin_init("LL", ANALOG);

    s->v1 =
        pin_init("V1", ANALOG);

    s->v5 =
        pin_init("V5", ANALOG);


    /* Control pins */
    s->hr_in =
        pin_init("HR_IN", ANALOG);

    s->leadoff =
        pin_init(
            "LEADOFF",
            INPUT_PULLUP
        );


    /* Outputs */
    s->out =
        pin_init("OUT", ANALOG);

    s->beat =
        pin_init("BEAT", OUTPUT);


    /* Controls */
    s->attr_hr =
        attr_init(
            "heartRate",
            72
        );

    s->attr_wander =
        attr_init(
            "wanderAmp",
            0
        );

    s->attr_noise =
        attr_init(
            "noiseAmp",
            0
        );

    s->attr_mains =
        attr_init(
            "plAmp",
            0
        );

    s->attr_gain =
        attr_init(
            "inaGain",
            450
        );

    s->attr_contact =
        attr_init(
            "contactQuality",
            98
        );

    s->attr_cmrr =
        attr_init(
            "cmrrDb",
            100
        );


    /* Initial state */
    s->phase = 0.0f;
    s->wander_phase = 0.0f;
    s->mains_phase = 0.0f;
    s->lfsr = 0xACE1u;
    s->beat_state = false;
    s->beat_samples = 0;


    pin_dac_write(
        s->out,
        MID_RAIL
    );

    pin_write(
        s->beat,
        0
    );


    printf(
        "[ECG-AFE] Stable PQRSTU generator\n"
    );

    printf(
        "[ECG-AFE] HR control = 72 BPM\n"
    );

    printf(
        "[ECG-AFE] OUT -> GPIO34\n"
    );

    printf(
        "[ECG-AFE] BEAT -> GPIO26\n"
    );


    const timer_config_t config =
    {
        .callback = chip_timer_cb,
        .user_data = s
    };

    timer_t timer =
        timer_init(
            &config
        );

    timer_start(
        timer,
        SAMPLE_PERIOD,
        true
    );
}