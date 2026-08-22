// Wearable ECG simulator controller
// ESP32 reads the custom ECG AFE chip, drives the OLED trace, and prints CSV.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

#define ECG_ADC_PIN 34
#define HR_POT_PIN 35
#define BEAT_LED_PIN 2
#define LEADOFF_PIN 15

#define SAMPLE_RATE_HZ 500
#define SAMPLE_PERIOD_US 2000UL
#define WAVEFORM_SIZE 128

#define ADC_MAX 4095.0f
#define ADC_REF_V 3.3f
#define AFE_MID_RAIL_V 1.65f
#define AFE_GAIN 450.0f

#define ECG_MIN_UV -1800.0f
#define ECG_MAX_UV 2200.0f
#define R_THRESHOLD_UV 520.0f
#define R_REFRACTORY_MS 260UL

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float waveform[WAVEFORM_SIZE];
int waveformIndex = 0;

float baseline_uV = 0.0f;
float filtered_uV = 0.0f;
float heartRate = 72.0f;
float noiseRms_uV = 0.0f;

bool wasAboveR = false;
unsigned long lastBeatMs = 0;
unsigned long beatLedUntilMs = 0;
unsigned long lastSampleUs = 0;
unsigned long lastOledMs = 0;
uint8_t serialDivider = 0;

static float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static int readAveragedAdc(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / 4);
}

static float adcToAfeMicrovolts(int raw) {
  float voltage = ((float)raw / ADC_MAX) * ADC_REF_V;
  return ((voltage - AFE_MID_RAIL_V) * 1000000.0f) / AFE_GAIN;
}

static float readHeartRatePot() {
  int raw = readAveragedAdc(HR_POT_PIN);
  if (raw < 8) {
    return heartRate;
  }
  return 40.0f + ((float)raw / ADC_MAX) * 140.0f;
}

static void updateBeatDetector(float ecg_uV) {
  unsigned long now = millis();
  bool aboveR = ecg_uV > R_THRESHOLD_UV;

  if (aboveR && !wasAboveR && (now - lastBeatMs) > R_REFRACTORY_MS) {
    if (lastBeatMs > 0) {
      unsigned long rrMs = now - lastBeatMs;
      float instantBpm = 60000.0f / (float)rrMs;
      if (instantBpm >= 35.0f && instantBpm <= 190.0f) {
        heartRate = heartRate * 0.75f + instantBpm * 0.25f;
      }
    }
    lastBeatMs = now;
    beatLedUntilMs = now + 55;
  }

  wasAboveR = aboveR;
  digitalWrite(BEAT_LED_PIN, now < beatLedUntilMs ? HIGH : LOW);
}

static void updateNoiseEstimate(float ecg_uV) {
  static float acc = 0.0f;
  static int count = 0;

  float residual = ecg_uV - filtered_uV;
  acc += residual * residual;
  count++;

  if (count >= 100) {
    noiseRms_uV = sqrtf(acc / 100.0f);
    acc = 0.0f;
    count = 0;
  }
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
  display.print("HR");
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.print((int)(heartRate + 0.5f));
  display.setTextSize(1);
  display.setCursor(68, 7);
  display.print("BPM");
  display.setCursor(96, 7);
  display.print(noiseRms_uV < 35.0f ? "OK" : "NOISY");

  display.drawLine(0, 18, 127, 18, SSD1306_WHITE);
  display.drawLine(0, 43, 127, 43, SSD1306_WHITE);

  int previousY = -1;
  for (int x = 0; x < WAVEFORM_SIZE; x++) {
    int idx = (waveformIndex + x) % WAVEFORM_SIZE;
    float v = clampf(waveform[idx], ECG_MIN_UV, ECG_MAX_UV);
    int y = 43 - (int)(v / 65.0f);
    y = constrain(y, 20, 62);

    if (previousY >= 0) {
      display.drawLine(x - 1, previousY, x, y, SSD1306_WHITE);
    }
    previousY = y;
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
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("ECG SIMULATOR");
  display.setCursor(0, 16);
  display.print("Lead II P-QRS-T");
  display.setCursor(0, 32);
  display.print("AFE + filters active");
  display.display();
  delay(900);

  for (int i = 0; i < WAVEFORM_SIZE; i++) {
    waveform[i] = 0.0f;
  }
}

void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_PERIOD_US) {
    return;
  }
  lastSampleUs += SAMPLE_PERIOD_US;

  bool leadOff = digitalRead(LEADOFF_PIN) == LOW;
  float potBpm = readHeartRatePot();
  if (lastBeatMs == 0) {
    heartRate = potBpm;
  }

  int ecgRaw = readAveragedAdc(ECG_ADC_PIN);
  float ecg_uV = adcToAfeMicrovolts(ecgRaw);

  baseline_uV += 0.0008f * (ecg_uV - baseline_uV);
  float centered_uV = ecg_uV - baseline_uV;
  filtered_uV += 0.55f * (centered_uV - filtered_uV);
  filtered_uV = clampf(filtered_uV, ECG_MIN_UV, ECG_MAX_UV);

  if (leadOff) {
    waveform[waveformIndex] = 0.0f;
    digitalWrite(BEAT_LED_PIN, LOW);
    wasAboveR = false;
  } else {
    waveform[waveformIndex] = filtered_uV;
    updateBeatDetector(filtered_uV);
    updateNoiseEstimate(centered_uV);
  }

  waveformIndex = (waveformIndex + 1) % WAVEFORM_SIZE;

  serialDivider++;
  if (serialDivider >= 5) {
    serialDivider = 0;
    Serial.print(filtered_uV, 1);
    Serial.print(',');
    Serial.println(heartRate, 1);
  }

  if (millis() - lastOledMs >= 50) {
    lastOledMs = millis();
    drawOled(leadOff);
  }
}
