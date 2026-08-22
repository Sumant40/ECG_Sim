// ECG Belt Simulator — software-generated PQRST waveform.
// Generates the ECG signal directly in firmware so the OLED and Serial
// plotter always show a proper ECG regardless of chip DAC simulation.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT    64
#define OLED_ADDR      0x3C

#define HR_POT_PIN      35   // potentiometer → heart-rate
#define BEAT_LED_PIN     2   // green LED flashes on each beat
#define LEADOFF_PIN     15   // push-button → lead-off (active LOW)

#define SAMPLE_RATE_HZ      500
#define SAMPLE_PERIOD_US   2000UL
#define WAVEFORM_SIZE       128

// ECG waveform amplitudes in mV (Lead-II)
#define P_AMP      40.0f
#define Q_AMP     -30.0f
#define R_AMP     700.0f
#define S_AMP     -80.0f
#define T_AMP     130.0f

#define R_THRESHOLD_MV    350.0f
#define R_REFRACTORY_MS   260UL

#define ECG_DISPLAY_MIN  -120.0f
#define ECG_DISPLAY_MAX   800.0f

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float waveform[WAVEFORM_SIZE];
int   waveformIndex = 0;

float ecgPhase  = 0.0f;   // 0…1, fraction through cardiac cycle
float heartRate = 72.0f;

bool          wasAboveR      = false;
unsigned long lastBeatMs     = 0;
unsigned long beatLedUntilMs = 0;

unsigned long lastSampleUs = 0;
unsigned long lastOledMs   = 0;
uint8_t       serialDiv    = 0;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Wrapped Gaussian on a cyclic 0…1 axis.
static float gaussian(float phase, float center, float sigma, float amp) {
  float d = phase - center;
  if (d >  0.5f) d -= 1.0f;
  if (d < -0.5f) d += 1.0f;
  return amp * expf(-(d * d) / (2.0f * sigma * sigma));
}

// Returns ECG amplitude in mV for phase in [0, 1).
static float leadII_mV(float ph) {
  float p =  gaussian(ph, 0.185f, 0.028f, P_AMP);
  float q =  gaussian(ph, 0.385f, 0.009f, Q_AMP);
  float r =  gaussian(ph, 0.407f, 0.010f, R_AMP);
  float s =  gaussian(ph, 0.432f, 0.012f, S_AMP);
  float t =  gaussian(ph, 0.670f, 0.070f, T_AMP);
  return p + q + r + s + t;
}

// Read heart-rate from potentiometer (40–180 BPM).
static float readHeartRatePot() {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(HR_POT_PIN);
  int raw = (int)(sum >> 2);
  if (raw < 8) return heartRate;
  return 40.0f + ((float)raw / 4095.0f) * 140.0f;
}

static void updateBeatDetector(float ecg_mV) {
  unsigned long now = millis();
  bool above = (ecg_mV > R_THRESHOLD_MV);

  if (above && !wasAboveR && (now - lastBeatMs) > R_REFRACTORY_MS) {
    if (lastBeatMs > 0) {
      float bpm = 60000.0f / (float)(now - lastBeatMs);
      if (bpm >= 35.0f && bpm <= 200.0f)
        heartRate = heartRate * 0.85f + bpm * 0.15f;
    }
    lastBeatMs     = now;
    beatLedUntilMs = now + 80;
  }
  wasAboveR = above;
  digitalWrite(BEAT_LED_PIN, (now < beatLedUntilMs) ? HIGH : LOW);
}

static void drawOled(bool leadOff) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (leadOff) {
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.print("LEAD OFF");
    display.setTextSize(1);
    display.setCursor(16, 48);
    display.print("Check electrodes");
    display.display();
    return;
  }

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("HR:");
  display.setTextSize(2);
  display.setCursor(22, 0);
  display.print((int)(heartRate + 0.5f));
  display.setTextSize(1);
  display.setCursor(70, 7);
  display.print("BPM");
  display.setCursor(98, 0);
  display.print("Lead II");

  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);

  // ECG trace area: pixels 18–63 (46 px tall).
  // Centre baseline at y=42; scale so 700 mV R-peak → ~28 px up.
  const float scale = 0.040f;   // px per mV
  const int   yBase = 42;
  int prevY = -1;
  for (int x = 0; x < WAVEFORM_SIZE; x++) {
    int idx = (waveformIndex + x) % WAVEFORM_SIZE;
    float v = clampf(waveform[idx], ECG_DISPLAY_MIN, ECG_DISPLAY_MAX);
    int y   = yBase - (int)(v * scale);
    y = constrain(y, 18, 63);
    if (prevY >= 0) display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);
    prevY = y;
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  pinMode(BEAT_LED_PIN, OUTPUT);
  digitalWrite(BEAT_LED_PIN, LOW);
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
  display.setCursor(0, 14); display.print("Lead II  PQRST");
  display.setCursor(0, 28); display.print("Software ECG gen");
  display.setCursor(0, 42); display.print("Initialising...");
  display.display();
  delay(700);

  for (int i = 0; i < WAVEFORM_SIZE; i++) waveform[i] = 0.0f;
  lastSampleUs = micros();
}

void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_PERIOD_US) return;
  lastSampleUs += SAMPLE_PERIOD_US;

  bool leadOff = (digitalRead(LEADOFF_PIN) == LOW);

  // Update heart rate from potentiometer.
  float potBpm = readHeartRatePot();
  if (lastBeatMs == 0) heartRate = potBpm;  // before first beat, trust pot

  if (leadOff) {
    waveform[waveformIndex] = 0.0f;
    digitalWrite(BEAT_LED_PIN, LOW);
    wasAboveR = false;
    ecgPhase  = 0.0f;
  } else {
    // Advance ECG phase by one sample tick.
    ecgPhase += heartRate / (60.0f * SAMPLE_RATE_HZ);
    if (ecgPhase >= 1.0f) ecgPhase -= 1.0f;

    // Generate ECG sample from Gaussian PQRST model.
    float ecg_mV = leadII_mV(ecgPhase);

    waveform[waveformIndex] = ecg_mV;
    updateBeatDetector(ecg_mV);

    // Serial plotter: output every 5th sample (~100 Hz).
    serialDiv++;
    if (serialDiv >= 5) {
      serialDiv = 0;
      Serial.print(ecg_mV, 1);
      Serial.print(',');
      Serial.println(heartRate, 1);
    }
  }

  waveformIndex = (waveformIndex + 1) % WAVEFORM_SIZE;

  if (millis() - lastOledMs >= 50) {
    lastOledMs = millis();
    drawOled(leadOff);
  }
}