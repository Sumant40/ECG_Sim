// ============================================================================
// WEARABLE ECG BELT — REALISTIC ECG SIMULATION
// Wokwi: ESP32 + SSD1306 + 2x Potentiometers + Green LED + Red Button
//
// Serial Plotter output: microvolts (µV), matching MIT-BIH scale
// Simulates: skin electrode → HPF → INA (×100) → LPF → 50 Hz notch → ADC
//
// R-peak: ~1500 µV  (Lead I, dry electrode, post-amplification)
// P-wave:  ~150 µV
// T-wave:  ~300 µV
// Normal sinus rhythm: 60–100 BPM default
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ── OLED ─────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR    0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── PINS ─────────────────────────────────────────────────────────────────────
#define ECG_POT_PIN    34   // baseline wander control
#define HR_POT_PIN     35   // heart rate 40–180 BPM
#define BEAT_LED_PIN    2
#define LEADOFF_PIN    15

// ── SAMPLE RATE ──────────────────────────────────────────────────────────────
#define SAMPLE_RATE        500          // Hz — AHA minimum for rhythm monitoring
#define SAMPLE_PERIOD_US  2000UL        // 1/500 Hz in microseconds
#define WAVEFORM_SIZE      128

// ── PHYSIOLOGICAL CONSTANTS (real ECG values) ────────────────────────────────
// All amplitudes in µV at the electrode surface (before amplification)
// INA gain = 100× is applied in software to get post-amp µV
// R-peak ~1500 µV post-gain is typical for Lead I dry electrode
//
//  Wave     Duration (ms)   Amplitude µV (electrode)   After ×100
//  P-wave      80–120 ms         15 µV                  1500 µV
//  PR interval 120–200 ms        (isoelectric)
//  QRS         60–100 ms         10 µV (R only)         1000–2000 µV
//  ST segment  80–120 ms         ~0 µV (flat)
//  T-wave      160 ms            3–5 µV                 300–500 µV
//
// Gaussian σ mapped to fraction of cardiac cycle:
//   At 75 BPM → 1 cycle = 800 ms = 400 samples at 500 Hz
//   P-wave width 100 ms → σ = 100/800 = 0.125 in phase units
//   QRS width 80 ms    → σ = 80/800  = 0.100 in phase units
//   T-wave width 160 ms→ σ = 160/800 = 0.200 in phase units

// ── PQRST WAVEFORM GENERATOR ─────────────────────────────────────────────────
// Returns electrode-level amplitude in µV (before amplification)
float gaussian(float p, float center, float sigma, float amplitude_uV) {
  float d = p - center;
  return amplitude_uV * expf(-(d * d) / (2.0f * sigma * sigma));
}

float generateECG_uV(float p) {
  // P-wave: atrial depolarisation, upright in Lead I
  float P = gaussian(p, 0.14f, 0.020f,  15.0f);

  // PR segment: isoelectric (flat) — nothing to add

  // Q-wave: small negative deflection before R
  float Q = -gaussian(p, 0.42f, 0.008f,  3.0f);

  // R-wave: main ventricular depolarisation peak
  // Real Lead I R-peak: 5–20 mV at limb leads; ~10 µV at chest for this lead config
  float R =  gaussian(p, 0.45f, 0.009f, 10.0f);

  // S-wave: negative deflection after R
  float S = -gaussian(p, 0.48f, 0.009f,  4.0f);

  // ST segment: isoelectric — tiny positive offset (~0.5 µV = normal variant)
  // Modelled as very wide, low Gaussian centred at 0.56
  float ST = gaussian(p, 0.56f, 0.030f,  0.5f);

  // T-wave: ventricular repolarisation, upright in Lead I
  float T =  gaussian(p, 0.67f, 0.038f,  3.5f);

  // U-wave: small, sometimes visible, not always present
  float U =  gaussian(p, 0.80f, 0.018f,  0.8f);

  return P + Q + R + S + ST + T + U;
}

// ── ANALOG FILTER CHAIN (SOFTWARE SIMULATION) ────────────────────────────────
//
// Real signal path:
//   Skin → HPF (0.5 Hz) → INA ×100 → LPF (150 Hz) → 50 Hz notch → ADC
//
// We simulate each stage as a software IIR filter applied to the signal.
//
// All filters use Direct Form II Transposed biquad sections.
// Coefficients calculated offline with scipy.signal.butter / iirnotch at 500 Hz.

struct Biquad
{
  float b0;
  float b1;
  float b2;

  float a1;
  float a2;

  float w1;
  float w2;

  Biquad(
    float _b0,
    float _b1,
    float _b2,
    float _a1,
    float _a2
  )
  {
    b0 = _b0;
    b1 = _b1;
    b2 = _b2;

    a1 = _a1;
    a2 = _a2;

    w1 = 0.0f;
    w2 = 0.0f;
  }

  float process(float x)
  {
    float y =
      b0 * x + w1;

    w1 =
      b1 * x
      - a1 * y
      + w2;

    w2 =
      b2 * x
      - a2 * y;

    return y;
  }

  void reset()
  {
    w1 = 0.0f;
    w2 = 0.0f;
  }
};

// ── STAGE 1: HIGH-PASS FILTER (HPF) — removes baseline wander ───────────────
// Butterworth 2nd order, fc = 0.5 Hz, fs = 500 Hz
// Removes DC drift and slow respiratory baseline wander
// scipy: butter(2, 0.5/250, btype='high')
Biquad hpf = { 0.99875078f, -1.99750156f, 0.99875078f,
               -1.99750622f, 0.99749690f };

// ── STAGE 2: INSTRUMENTATION AMPLIFIER (INA) GAIN ────────────────────────────
// ADS1292R PGA gain = 6 (set via SPI register CONFIG1 in real hardware)
// Combined with internal reference, effective voltage gain ≈ 6× at ADC input
// We use ×6 here so total with the raw 10 µV signal → ~60 µV at ADC
// Then we scale ×1000 for display in µV output
#define INA_GAIN  6.0f

// ── STAGE 3: LOW-PASS FILTER (LPF) — anti-aliasing + EMG rejection ──────────
// Butterworth 4th order, fc = 40 Hz, fs = 500 Hz
// Two cascaded biquad sections
// scipy: butter(4, 40/250, btype='low') → sos form
Biquad lpf1 = { 0.00048845f, 0.00097690f, 0.00048845f,
                -1.76004837f, 0.77296192f };
Biquad lpf2 = { 1.00000000f, 2.00000000f, 1.00000000f,
                -1.90456724f, 0.91487003f };

// ── STAGE 4: 50 Hz NOTCH FILTER (power-line interference rejection) ──────────
// IIR notch, Q = 30, f0 = 50 Hz, fs = 500 Hz
// scipy: iirnotch(50, 30, 500)
// In India: 50 Hz mains; in US/Japan use 60 Hz variant
Biquad notch50 = { 0.96940026f, -1.56258775f, 0.96940026f,
                   -1.56258775f, 0.93880051f };

// ── NOISE GENERATORS ─────────────────────────────────────────────────────────
// Simple LCG pseudo-random noise (no stdlib rand() needed on ESP32 bare metal)
uint32_t lfsr = 0xACE1u;
float whitenoise(float amplitude_uV) {
  lfsr ^= lfsr << 13;
  lfsr ^= lfsr >> 17;
  lfsr ^= lfsr <<  5;
  return ((float)(lfsr & 0xFFFF) / 32768.0f - 1.0f) * amplitude_uV;
}

// Baseline wander: slow sinusoidal drift at respiratory rate (~0.25 Hz)
// Simulates breathing-induced thoracic impedance changes
float wanderPhase = 0.0f;
float baselineWander_uV(float amount_uV) {
  wanderPhase += (2.0f * M_PI * 0.25f) / SAMPLE_RATE;
  if (wanderPhase > 2.0f * M_PI) wanderPhase -= 2.0f * M_PI;
  return sinf(wanderPhase) * amount_uV;
}

// 50 Hz power-line interference (what the notch filter must remove)
float plPhase = 0.0f;
float powerline50Hz_uV(float amplitude_uV) {
  plPhase += (2.0f * M_PI * 50.0f) / SAMPLE_RATE;
  if (plPhase > 2.0f * M_PI) plPhase -= 2.0f * M_PI;
  return sinf(plPhase) * amplitude_uV;
}

// ── ECG STATE ─────────────────────────────────────────────────────────────────
float phase       = 0.0f;
float heartRate   = 72.0f;
float waveform[WAVEFORM_SIZE];
int   waveformIndex = 0;

// ── BEAT DETECTION (R-wave phase crossing) ────────────────────────────────────
bool         prevRregion = false;
unsigned long beatTime   = 0;

// ── TIMING ───────────────────────────────────────────────────────────────────
unsigned long lastSampleTime  = 0;
unsigned long lastOLEDUpdate  = 0;
int           plotCounter     = 0;

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(BEAT_LED_PIN, OUTPUT);
  digitalWrite(BEAT_LED_PIN, LOW);
  pinMode(LEADOFF_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,  0); display.println("WEARABLE ECG BELT");
  display.setCursor(0, 14); display.println("Realistic PQRST");
  display.setCursor(0, 28); display.println("R-peak ~1500 uV");
  display.setCursor(0, 42); display.println("50Hz notch active");
  display.display();
  delay(1200);

  for (int i = 0; i < WAVEFORM_SIZE; i++) waveform[i] = 0.0f;
}

// ── DRAW OLED ─────────────────────────────────────────────────────────────────
void drawOLED(bool leadOff, float rms_uV) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (leadOff) {
    display.setTextSize(2);
    display.setCursor(8, 22);
    display.println("LEAD OFF");
    display.setTextSize(1);
    display.setCursor(18, 48);
    display.println("Check electrodes");
    display.display();
    return;
  }

  // Header: HR + signal quality
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("HR:");
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.print((int)heartRate);
  display.setTextSize(1);
  display.setCursor(68, 7);
  display.print("BPM");

  // RMS noise indicator (right side of header)
  display.setCursor(88, 0);
  display.print(rms_uV < 50.0f ? "SNR:OK" : "NOISE!");

  // Divider
  display.drawLine(0, 18, 127, 18, SSD1306_WHITE);

  // ECG waveform (y range: 19–63)
  int prevY = -1;
  for (int x = 0; x < WAVEFORM_SIZE; x++) {
    int idx = (waveformIndex + x) % WAVEFORM_SIZE;
    // waveform[] stores filtered µV. Scale: 1500 µV R-peak fits in 22 pixels
    float v = waveform[idx];
    // Centre at y=41, scale: 1500 µV → 20 pixels up
    int y = 41 - (int)(v / 75.0f);
    y = constrain(y, 20, 62);
    if (prevY >= 0) display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);
    prevY = y;
  }
  display.display();
}

// ── MAIN LOOP ─────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now_us = micros();
  if (now_us - lastSampleTime < SAMPLE_PERIOD_US) return;
  lastSampleTime = now_us;

  // ── Lead-off ──────────────────────────────────────────────────────────────
  bool leadOff = (digitalRead(LEADOFF_PIN) == LOW);

  // ── Heart rate potentiometer (40–180 BPM) ─────────────────────────────────
  int hrRaw   = constrain(analogRead(HR_POT_PIN), 0, 4095);
  heartRate   = 40.0f + ((float)hrRaw / 4095.0f) * 140.0f;
  float phaseInc = heartRate / (60.0f * SAMPLE_RATE);
  phase += phaseInc;
  if (phase >= 1.0f) phase -= 1.0f;

  // ── ECG potentiometer → baseline wander amplitude control ────────────────
  int ecgRaw = analogRead(ECG_POT_PIN);
  // Map 0–4095 to 0–200 µV wander amplitude
  float wanderAmp = ((float)ecgRaw / 4095.0f) * 200.0f;

  // ── Stage 0: Generate raw electrode signal (µV) ───────────────────────────
  // This is what a stainless steel dry electrode picks up from the skin
  float raw_uV = generateECG_uV(phase);

  // Add physiological noise components:
  //   50 Hz power-line: ~50 µV (strong near mains; HPF/notch must remove)
  //   EMG artifact: ~5 µV white noise (muscle electrical activity)
  //   Baseline wander: controlled by ECG pot (0–200 µV amplitude)
  float noise_uV    = powerline50Hz_uV(50.0f) + whitenoise(5.0f);
  float wander_uV   = baselineWander_uV(wanderAmp);
  float dirty_uV    = raw_uV + noise_uV + wander_uV;

  // ── Stage 1: HPF — removes baseline wander ────────────────────────────────
  float hpf_out = hpf.process(dirty_uV);

  // ── Stage 2: INA gain ×6 (ADS1292R PGA) ──────────────────────────────────
  // In real hardware the gain is set via SPI register; here it's just a multiply
  float amp_uV = hpf_out * INA_GAIN;

  // ── Stage 3: LPF 40 Hz (two cascaded biquads) ────────────────────────────
  float lpf_out = lpf2.process(lpf1.process(amp_uV));

  // ── Stage 4: 50 Hz notch ─────────────────────────────────────────────────
  float filt_uV = notch50.process(lpf_out);

  // ── RMS noise estimate (rolling, 100 samples) ────────────────────────────
  static float rms_acc = 0.0f;
  static int   rms_n   = 0;
  static float rms_out = 0.0f;
  rms_acc += filt_uV * filt_uV;
  rms_n++;
  if (rms_n >= 100) {
    rms_out = sqrtf(rms_acc / 100.0f);
    rms_acc = 0.0f;
    rms_n   = 0;
  }

  if (leadOff) {
    digitalWrite(BEAT_LED_PIN, LOW);
    waveform[waveformIndex] = 0.0f;
  } else {
    // ── Store filtered signal for OLED ──────────────────────────────────────
    waveform[waveformIndex] = filt_uV;

    // ── R-wave beat detection (phase-crossing method) ────────────────────────
    // Detects rising edge of R-peak region (phase 0.435–0.465)
    // In real firmware: use Pan-Tompkins on filtered signal instead
    bool RRegion = (phase >= 0.435f && phase < 0.465f);
    if (RRegion && !prevRregion) {
      digitalWrite(BEAT_LED_PIN, HIGH);
      beatTime = millis();
    }
    prevRregion = RRegion;

    if (digitalRead(BEAT_LED_PIN) == HIGH && millis() - beatTime >= 50)
      digitalWrite(BEAT_LED_PIN, LOW);
  }

  waveformIndex = (waveformIndex + 1) % WAVEFORM_SIZE;

  // ============================================================
 // SERIAL PLOTTER
 // ============================================================

// ============================================================
// SERIAL PLOTTER
// ============================================================
//
// Send only the variables that actually exist in this program.
//
// First trace = ECG signal
// Second trace = heart rate
//
// ============================================================

 static uint8_t plotCounter = 0;

 plotCounter++;

 if (plotCounter >= 5)
 {
    plotCounter = 0;

    Serial.print(ecgRaw, 2);
    Serial.print(",");
    Serial.println(heartRate, 2);
  }

  // ── OLED refresh @ 20 fps ────────────────────────────────────────────────
  if (millis() - lastOLEDUpdate >= 50) {
    lastOLEDUpdate = millis();
    drawOLED(leadOff, rms_out);
  }
}