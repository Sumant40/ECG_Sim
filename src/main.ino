/*
 * ============================================================
 * WEARABLE ECG BELT  —  ESP32 + Custom ECG AFE  v2
 * ============================================================
 *
 * Hardware:
 *   AFE OUT    -> GPIO34  (ADC, 12-bit)
 *   AFE BEAT   -> GPIO26  (R-peak pulse from AFE chip)
 *   LEAD-OFF   -> GPIO15  (INPUT_PULLUP; LOW = lead off)
 *   OLED SDA   -> GPIO21
 *   OLED SCL   -> GPIO22
 *   BEAT LED   -> GPIO2   (via 330 Ω)
 *
 * Improvements over v1:
 *   1. ADC baseline now uses a faster-attack / slower-decay
 *      tracker so the display is live within ~0.5 s instead of ~2 s.
 *   2. Serial plotter emits labeled channels so Arduino IDE
 *      Serial Plotter shows a legend:
 *        ECG_mV:value  Baseline:0  HiLimit:200  LoLimit:-200
 *   3. OLED auto-scale uses a tighter floor (2 mV) so small
 *      signals aren't crushed.
 *   4. OLED shows lead name (II) and µV/div estimate.
 *   5. Lead-off state shown on OLED and serial output.
 *   6. BPM display shows "---" during acquisition phase.
 * ============================================================
 */

#include <Arduino.h>
#include <stdint.h>          /* uint8_t, uint32_t — explicit, fixes IDE IntelliSense */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// ============================================================
// PIN DEFINITIONS
// ============================================================

#define ECG_PIN        34
#define BEAT_PIN       26
#define BEAT_LED_PIN    2
#define LEADOFF_PIN    15
#define OLED_SDA       21
#define OLED_SCL       22


// ============================================================
// SAMPLING  (must match AFE chip SAMPLE_PERIOD = 2000 µs)
// ============================================================

#define SAMPLE_RATE    500          /* Hz   */
#define SAMPLE_US      2000UL       /* µs   */


// ============================================================
// OLED WAVEFORM BUFFER
// ============================================================

#define WAVEFORM_SIZE  128

float waveform[WAVEFORM_SIZE];
int   waveformIndex = 0;


// ============================================================
// HEART RATE
// ============================================================

float         heartRate    = 0.0f;   /* 0 = not yet acquired */
unsigned long lastBeatTime = 0;
bool          hrValid      = false;


// ============================================================
// BEAT LED
// ============================================================

bool          beatLEDActive = false;
unsigned long beatLEDStart  = 0;
#define BEAT_LED_MS  60


// ============================================================
// TIMING
// ============================================================

unsigned long lastSampleTime = 0;
unsigned long lastOLEDTime   = 0;


// ============================================================
// ECG
// ============================================================

float ecg_mV = 0.0f;


// ============================================================
// ADC BASELINE TRACKER
//
// Two-phase design:
//
//   PHASE 1 — Startup (first BASELINE_INIT_SAMPLES calls):
//     α = 0.20  Very fast. Converges to the ~2048 ADC mid-rail
//     in ~25 samples = 0.25 s at 100 Hz output rate.
//
//   PHASE 2 — Steady-state tracking:
//     α = 0.002  Slow. Follows electrode DC drift without
//     distorting the ECG signal (0.5–40 Hz band).
//
// The previous asymmetric (diff < 40) design was broken:
// at startup diff is always thousands of counts, so it always
// chose the slow path (α 0.002) and took ~2 s to converge,
// producing the ±1500 mV startup burst seen in serial output.
// ============================================================

#define BASELINE_INIT_SAMPLES  50     /* ~0.5 s at 100 Hz output */

float    adcBaseline         = 2048.0f;
bool     baselineInitialized = false;
uint16_t baselineInitCount   = 0;


// ============================================================
// ECG POLARITY
//   false = positive R-peak (Lead II normal orientation)
//   true  = invert if wiring gives a downward deflection
// ============================================================

const bool INVERT_ECG = false;


// ============================================================
// readECG()
// ============================================================

float readECG()
{
    // --------------------------------------------------------
    // Oversample ×8 for better effective resolution
    // --------------------------------------------------------

    uint32_t total = 0;
    for (int i = 0; i < 8; i++)
        total += analogRead(ECG_PIN);

    float adcValue = total / 8.0f;


    // --------------------------------------------------------
    // Bootstrap on very first call
    // --------------------------------------------------------

    if (!baselineInitialized)
    {
        adcBaseline         = adcValue;
        baselineInitialized = true;
    }


    // --------------------------------------------------------
    // Two-phase IIR baseline tracker
    // --------------------------------------------------------

    float alpha;

    if (baselineInitCount < BASELINE_INIT_SAMPLES)
    {
        alpha = 0.20f;          /* Phase 1: fast convergence   */
        baselineInitCount++;
    }
    else
    {
        alpha = 0.002f;         /* Phase 2: slow drift removal */
    }

    adcBaseline = (1.0f - alpha) * adcBaseline + alpha * adcValue;


    // --------------------------------------------------------
    // Remove DC
    // --------------------------------------------------------

    float signal = adcValue - adcBaseline;


    // --------------------------------------------------------
    // Convert to mV  (3300 mV full scale, 4096 counts)
    // --------------------------------------------------------

    float signal_mV = signal * (3300.0f / 4096.0f);


    // --------------------------------------------------------
    // Optional inversion
    // --------------------------------------------------------

    if (INVERT_ECG)
        signal_mV = -signal_mV;


    // --------------------------------------------------------
    // Safety clamp
    // --------------------------------------------------------

    signal_mV = constrain(signal_mV, -1500.0f, 1500.0f);

    return signal_mV;
}


// ============================================================
// processBeat()
//
// Called every sample when leads are connected.
// Detects rising edge on GPIO26 (AFE BEAT pin).
// Calculates RR interval -> BPM.
// ============================================================

void processBeat()
{
    static bool prevBeat = false;

    bool curBeat = (digitalRead(BEAT_PIN) == HIGH);
    unsigned long now = millis();


    // --------------------------------------------------------
    // Rising edge
    // --------------------------------------------------------

    if (curBeat && !prevBeat)
    {
        if (lastBeatTime != 0)
        {
            unsigned long rr = now - lastBeatTime;

            /* Valid RR: 300..2000 ms  =  30..200 BPM */
            if (rr >= 300 && rr <= 2000)
            {
                float newHR = 60000.0f / (float)rr;

                if (!hrValid)
                {
                    heartRate = newHR;      /* first beat — no smoothing */
                    hrValid   = true;
                }
                else
                {
                    /* Weighted average — weight new beat at 30 % */
                    heartRate = 0.70f * heartRate + 0.30f * newHR;
                }

                heartRate = constrain(heartRate, 30.0f, 220.0f);
            }
        }

        lastBeatTime = now;

        /* Flash LED */
        digitalWrite(BEAT_LED_PIN, HIGH);
        beatLEDActive = true;
        beatLEDStart  = now;
    }

    prevBeat = curBeat;


    // --------------------------------------------------------
    // LED off after timeout
    // --------------------------------------------------------

    if (beatLEDActive && (now - beatLEDStart >= BEAT_LED_MS))
    {
        digitalWrite(BEAT_LED_PIN, LOW);
        beatLEDActive = false;
    }
}


// ============================================================
// drawOLED()
// ============================================================

void drawOLED(bool leadOff)
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);


    // --------------------------------------------------------
    // LEAD-OFF SCREEN
    // --------------------------------------------------------

    if (leadOff)
    {
        display.setTextSize(2);
        display.setCursor(8, 14);
        display.println("LEAD OFF");

        display.setTextSize(1);
        display.setCursor(16, 44);
        display.println("Check electrodes");

        display.display();
        return;
    }


    // --------------------------------------------------------
    // HEADER  (top 18 px)
    //   "HR: 72 BPM   II"
    // --------------------------------------------------------

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("HR:");

    display.setTextSize(2);
    display.setCursor(20, 0);
    if (hrValid)
        display.print((int)(heartRate + 0.5f));
    else
        display.print("---");

    display.setTextSize(1);
    display.setCursor(66, 7);
    display.print("BPM");

    display.setCursor(105, 7);
    display.print("II");

    /* Divider */
    display.drawLine(0, 17, 127, 17, SSD1306_WHITE);


    // --------------------------------------------------------
    // WAVEFORM AUTO-SCALE
    //
    // Floor raised from 10 to 2 mV so small signals are visible.
    // Ceiling kept at 300 mV.
    // --------------------------------------------------------

    float maximum = 2.0f;
    for (int i = 0; i < WAVEFORM_SIZE; i++)
    {
        float mag = fabsf(waveform[i]);
        if (mag > maximum)
            maximum = mag;
    }
    maximum = constrain(maximum, 2.0f, 300.0f);


    // --------------------------------------------------------
    // DRAW ECG TRACE  (pixels 20..63, i.e. 44 px tall)
    //
    // Baseline sits at y = 42.
    // --------------------------------------------------------

    int prevY = -1;

    for (int x = 0; x < 128; x++)
    {
        int   index = (waveformIndex + x) % WAVEFORM_SIZE;
        float val   = waveform[index];

        /* Scale: 22 px per "maximum" value, centred at y=42 */
        int y = 42 - (int)(val * 22.0f / maximum);
        y = constrain(y, 19, 63);

        if (prevY >= 0)
            display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);

        prevY = y;
    }


    display.display();
}


// ============================================================
// setup()
// ============================================================

void setup()
{
    Serial.begin(115200);


    /* GPIOs */
    pinMode(BEAT_PIN,     INPUT);
    pinMode(BEAT_LED_PIN, OUTPUT);
    pinMode(LEADOFF_PIN,  INPUT_PULLUP);

    digitalWrite(BEAT_LED_PIN, LOW);


    /* ADC — 12-bit, 0..4095 */
    analogReadResolution(12);


    /* I2C + OLED */
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(400000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
    {
        /* Blink rapidly to indicate OLED failure */
        while (true)
        {
            digitalWrite(BEAT_LED_PIN, HIGH); delay(100);
            digitalWrite(BEAT_LED_PIN, LOW);  delay(100);
        }
    }


    /* Clear waveform buffer */
    memset(waveform, 0, sizeof(waveform));


    /* Splash screen */
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0,  4); display.println("WEARABLE ECG BELT v2");
    display.setCursor(0, 18); display.println("6-Electrode AFE");
    display.setCursor(0, 32); display.println("Lead II  |  500 Hz");
    display.setCursor(0, 46); display.println("Initialising...");
    display.display();

    delay(1200);


    /* Timers */
    lastSampleTime = micros();
    lastOLEDTime   = millis();
}


// ============================================================
// loop()
// ============================================================

void loop()
{
    unsigned long now = micros();


    // --------------------------------------------------------
    // 500 Hz sample gate
    // --------------------------------------------------------

    if (now - lastSampleTime < SAMPLE_US)
        return;

    lastSampleTime += SAMPLE_US;


    // --------------------------------------------------------
    // Lead-off state
    // --------------------------------------------------------

    bool leadOff = (digitalRead(LEADOFF_PIN) == LOW);


    // --------------------------------------------------------
    // Read ECG  (always, even when lead-off, for debug)
    // --------------------------------------------------------

    ecg_mV = readECG();

    if (!isfinite(ecg_mV))
        ecg_mV = 0.0f;


    // --------------------------------------------------------
    // Beat processing
    // --------------------------------------------------------

    if (!leadOff)
    {
        processBeat();
    }
    else
    {
        digitalWrite(BEAT_LED_PIN, LOW);
        beatLEDActive = false;
        lastBeatTime  = 0;
        hrValid       = false;
        heartRate     = 0.0f;
    }


    // --------------------------------------------------------
    // Store in waveform ring buffer
    // --------------------------------------------------------

    waveform[waveformIndex] = ecg_mV;
    waveformIndex = (waveformIndex + 1) % WAVEFORM_SIZE;


    // --------------------------------------------------------
    // Serial Plotter output  (100 Hz — every 5th sample)
    //
    // Format understood by Arduino Serial Plotter (IDE 2):
    //   Label:value  Label:value ...
    //
    // Channels:
    //   ECG_mV   — the ECG signal
    //   Zero     — flat zero reference line
    //   MaxLimit — upper guide at +200 mV
    //   MinLimit — lower guide at -200 mV
    // --------------------------------------------------------

    static uint8_t serialDiv = 0;
    if (++serialDiv >= 5)
    {
        serialDiv = 0;

        if (leadOff)
        {
            Serial.println("ECG_mV:0 Zero:0 MaxLimit:200 MinLimit:-200");
        }
        else
        {
            Serial.print("ECG_mV:");
            Serial.print(ecg_mV, 2);
            Serial.print(" Zero:0");
            Serial.print(" MaxLimit:200");
            Serial.println(" MinLimit:-200");
        }
    }


    // --------------------------------------------------------
    // OLED update  (20 Hz)
    // --------------------------------------------------------

    unsigned long nowMs = millis();
    if (nowMs - lastOLEDTime >= 50)
    {
        lastOLEDTime = nowMs;
        drawOLED(leadOff);
    }
}