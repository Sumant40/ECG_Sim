/*
 * ECG Belt — main firmware
 *
 * Reads filtered Lead-II ECG from the custom chip's OUT pin (GPIO34).
 * Beat detection uses the chip's hardware BEAT output (GPIO26).
 * Displays scrolling ECG trace + HR on SSD1306 OLED.
 * Lead-off button (GPIO15) freezes trace and shows warning.
 *
 * Signal chain handled entirely in chip:
 *   RA/LA/RL/LL/V1/V5 pots → INA → BPF 0.5-40 Hz → Notch 50 Hz → OUT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

/* ── Pin definitions ──────────────────────────────────────────────── */
#define ECG_ADC_PIN    34   /* chip OUT  → filtered Lead-II voltage   */
#define BEAT_IN_PIN    26   /* chip BEAT → digital R-peak pulse       */
#define BEAT_LED_PIN    2   /* green LED driven by chip BEAT pulse     */
#define LEADOFF_PIN    15   /* push-button, active LOW                 */

/* ── Sampling ─────────────────────────────────────────────────────── */
#define SAMPLE_RATE_HZ    500
#define SAMPLE_PERIOD_US  2000UL
#define WAVEFORM_SIZE     128

/* ── OLED ─────────────────────────────────────────────────────────── */
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

/* ── State ─────────────────────────────────────────────────────────── */
float         waveform[WAVEFORM_SIZE];
int           waveIdx       = 0;
float         heartRate     = 72.0f;

/* Beat tracking via hardware BEAT pin */
bool          prevBeatPin   = false;
unsigned long lastBeatMs    = 0;
unsigned long ledOffMs      = 0;

unsigned long lastSampleUs  = 0;
unsigned long lastOledMs    = 0;
uint8_t       serialDiv     = 0;

/* ── Helpers ──────────────────────────────────────────────────────── */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* ── Read chip OUT and convert to centred voltage ─────────────────── */
/* Chip outputs 0–3.3 V centred at 1.65 V (MID_RAIL).               */
/* Returns signal in volts, range roughly ±0.3 V for Lead-II.        */
static float readECG_V() {
  /* 4-sample average to reduce ESP32 ADC noise */
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(ECG_ADC_PIN);
  float adc = (float)(sum >> 2);
  return (adc / 4095.0f * 3.3f) - 1.65f;   /* centred at 0 V */
}

/* ── Handle chip BEAT pin → LED + HR update ──────────────────────── */
static void handleBeat() {
  bool beatNow = (digitalRead(BEAT_IN_PIN) == HIGH);
  unsigned long now = millis();

  /* Rising edge = new R-peak detected by chip hardware */
  if (beatNow && !prevBeatPin) {
    if (lastBeatMs > 0) {
      float rr_ms = (float)(now - lastBeatMs);
      if (rr_ms > 250.0f && rr_ms < 2000.0f) {
        float bpm = 60000.0f / rr_ms;
        heartRate = heartRate * 0.80f + bpm * 0.20f;  /* low-pass smooth */
      }
    }
    lastBeatMs = now;
    ledOffMs   = now + 80;
    digitalWrite(BEAT_LED_PIN, HIGH);
  }

  if (now >= ledOffMs) {
    digitalWrite(BEAT_LED_PIN, LOW);
  }
  prevBeatPin = beatNow;
}

/* ── OLED draw ─────────────────────────────────────────────────────── */
static void drawOled(bool leadOff) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (leadOff) {
    display.setTextSize(2);
    display.setCursor(8, 16);
    display.print("LEAD OFF");
    display.setTextSize(1);
    display.setCursor(12, 48);
    display.print("Check electrodes");
    display.display();
    return;
  }

  /* ── Top bar ─ HR value ───────────────────────────────────────── */
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("HR:");
  display.setTextSize(2);
  display.setCursor(22, 0);
  display.print((int)(heartRate + 0.5f));
  display.setTextSize(1);
  display.setCursor(70, 7);
  display.print("BPM");
  display.setCursor(96, 0);
  display.print("Lead II");

  /* ── Divider ──────────────────────────────────────────────────── */
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);

  /* ── ECG trace (y = 18..63, 46 px, baseline at y=42) ─────────── */
  /*    Chip output range ≈ ±0.3 V → scale: 1 V = 120 px           */
  const float px_per_volt = 120.0f;
  const int   yBase       = 42;
  int prevY = -1;
  for (int x = 0; x < WAVEFORM_SIZE; x++) {
    int   idx = (waveIdx + x) % WAVEFORM_SIZE;
    float v   = clampf(waveform[idx], -0.35f, 0.35f);
    int   y   = yBase - (int)(v * px_per_volt);
    y = constrain(y, 18, 63);
    if (prevY >= 0) display.drawLine(x-1, prevY, x, y, SSD1306_WHITE);
    prevY = y;
  }

  display.display();
}

/* ── setup ─────────────────────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);

  pinMode(BEAT_LED_PIN, OUTPUT);
  digitalWrite(BEAT_LED_PIN, LOW);
  pinMode(BEAT_IN_PIN, INPUT);
  pinMode(LEADOFF_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(100);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,  0); display.print("ECG BELT SIMULATOR");
  display.setCursor(0, 12); display.print("6-Electrode AFE Chip");
  display.setCursor(0, 24); display.print("RA  LA  RL  LL  V1  V5");
  display.setCursor(0, 36); display.print("INA > BPF > Notch > OUT");
  display.setCursor(0, 50); display.print("Initialising...");
  display.display();
  delay(1200);

  for (int i = 0; i < WAVEFORM_SIZE; i++) waveform[i] = 0.0f;
  lastSampleUs = micros();
}

/* ── loop ──────────────────────────────────────────────────────────── */
void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_PERIOD_US) return;
  lastSampleUs += SAMPLE_PERIOD_US;

  bool leadOff = (digitalRead(LEADOFF_PIN) == LOW);

  /* Always handle beat LED and HR update */
  handleBeat();

  if (leadOff) {
    waveform[waveIdx] = 0.0f;
    digitalWrite(BEAT_LED_PIN, LOW);
  } else {
    /* Read filtered ECG from chip OUT pin */
    float ecg_v = readECG_V();
    waveform[waveIdx] = ecg_v;

    /* Serial plotter: ECG (mV-scaled) | HR | lead-off sentinel */
    if (++serialDiv >= 5) {
      serialDiv = 0;
      Serial.print(ecg_v * 1000.0f, 1);   /* convert V → mV for readability */
      Serial.print(',');
      Serial.print(heartRate, 1);
      Serial.print(',');
      Serial.println(leadOff ? 0 : 300);  /* 300 = top-of-plotter sentinel   */
    }
  }

  waveIdx = (waveIdx + 1) % WAVEFORM_SIZE;

  /* OLED refresh at 20 Hz */
  if (millis() - lastOledMs >= 50) {
    lastOledMs = millis();
    drawOled(leadOff);
  }
}
