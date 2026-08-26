/*
 * ============================================================
 * WEARABLE ECG BELT
 * ESP32 + Custom ECG AFE + SSD1306 OLED
 * ============================================================
 *
 * AFE:
 *   OUT      -> GPIO34
 *   BEAT     -> GPIO26
 *   LEADOFF  -> GPIO15
 *
 * OLED:
 *   SDA      -> GPIO21
 *   SCL      -> GPIO22
 *
 * Beat LED:
 *   GPIO2 -> 330R -> LED -> GND
 *
 * Serial:
 *   One ECG value in mV per line.
 *   Designed for the Python live plotter.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ECG_PIN       34
#define BEAT_PIN      26
#define BEAT_LED_PIN   2
#define LEADOFF_PIN   15

#define OLED_SDA      21
#define OLED_SCL      22

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS  0x3C

#define SAMPLE_RATE   500
#define SAMPLE_US     2000UL

#define WAVEFORM_SIZE 128

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    -1
);

float waveform[WAVEFORM_SIZE] = {};
int waveformIndex = 0;

float ecg_mV = 0.0f;

float heartRate = 72.0f;
bool hrValid = false;

unsigned long lastBeatTime = 0;

bool beatLEDActive = false;
unsigned long beatLEDStart = 0;

#define BEAT_LED_MS 60

unsigned long lastSampleTime = 0;
unsigned long lastOLEDTime = 0;


/* ------------------------------------------------------------
 * Baseline tracker
 *
 * ESP32 sees a ~1.65 V centered signal.
 * The tracker removes only the DC component.
 * It is deliberately slow so it doesn't follow the ECG.
 * ------------------------------------------------------------ */
float adcBaseline = 2048.0f;
bool baselineInitialized = false;
uint16_t baselineInitCount = 0;

#define BASELINE_INIT_SAMPLES 250

float readECG()
{
    uint32_t total = 0;

    for (int i = 0; i < 8; i++)
    {
        total += analogRead(ECG_PIN);
    }

    float adcValue =
        total / 8.0f;


    if (!baselineInitialized)
    {
        adcBaseline = adcValue;
        baselineInitialized = true;
    }


    float alpha;

    if (baselineInitCount <
        BASELINE_INIT_SAMPLES)
    {
        /* ~0.5 s startup convergence at 500 Hz */
        alpha = 0.08f;
        baselineInitCount++;
    }
    else
    {
        /* ~4 s time constant */
        alpha = 0.0005f;
    }


    adcBaseline =
        (1.0f - alpha) *
        adcBaseline +
        alpha *
        adcValue;


    float signal =
        adcValue -
        adcBaseline;


    float signal_mV =
        signal *
        3300.0f /
        4096.0f;


    return constrain(
        signal_mV,
        -600.0f,
        600.0f
    );
}


/* ------------------------------------------------------------
 * Beat processing
 * ------------------------------------------------------------ */
void processBeat()
{
    static bool previousBeat =
        false;

    bool currentBeat =
        digitalRead(BEAT_PIN) == HIGH;

    unsigned long now =
        millis();


    if (
        currentBeat &&
        !previousBeat
    )
    {
        if (lastBeatTime != 0)
        {
            unsigned long rr =
                now -
                lastBeatTime;

            if (
                rr >= 300 &&
                rr <= 2000
            )
            {
                float bpm =
                    60000.0f /
                    (float)rr;

                if (!hrValid)
                {
                    heartRate = bpm;
                    hrValid = true;
                }
                else
                {
                    heartRate =
                        0.80f * heartRate +
                        0.20f * bpm;
                }

                heartRate =
                    constrain(
                        heartRate,
                        30.0f,
                        200.0f
                    );
            }
        }

        lastBeatTime = now;

        digitalWrite(
            BEAT_LED_PIN,
            HIGH
        );

        beatLEDActive = true;
        beatLEDStart = now;
    }


    previousBeat =
        currentBeat;


    if (
        beatLEDActive &&
        (
            now -
            beatLEDStart
            >=
            BEAT_LED_MS
        )
    )
    {
        digitalWrite(
            BEAT_LED_PIN,
            LOW
        );

        beatLEDActive = false;
    }
}


/* ------------------------------------------------------------
 * OLED
 * ------------------------------------------------------------ */
void drawOLED(bool leadOff)
{
    display.clearDisplay();

    display.setTextColor(
        SSD1306_WHITE
    );


    if (leadOff)
    {
        display.setTextSize(2);
        display.setCursor(8, 18);
        display.println("LEAD OFF");

        display.setTextSize(1);
        display.setCursor(20, 48);
        display.println("Release button");

        display.display();
        return;
    }


    /* Header */
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("HR:");

    display.setTextSize(2);
    display.setCursor(20, 0);

    if (hrValid)
        display.print(
            (int)(heartRate + 0.5f)
        );
    else
        display.print("---");

    display.setTextSize(1);
    display.setCursor(68, 7);
    display.print("BPM");

    display.setCursor(105, 7);
    display.print("II");


    display.drawLine(
        0,
        17,
        127,
        17,
        SSD1306_WHITE
    );


    /* Find signal magnitude */
    float maximum = 10.0f;

    for (
        int i = 0;
        i < WAVEFORM_SIZE;
        i++
    )
    {
        float a =
            fabsf(
                waveform[i]
            );

        if (a > maximum)
            maximum = a;
    }

    maximum =
        constrain(
            maximum,
            10.0f,
            350.0f
        );


    /* ECG baseline */
    const int baselineY = 43;

    int previousY = -1;

    for (
        int x = 0;
        x < 128;
        x++
    )
    {
        int index =
            (
                waveformIndex +
                x
            ) %
            WAVEFORM_SIZE;

        float value =
            waveform[index];


        int y =
            baselineY -
            (int)(
                value *
                20.0f /
                maximum
            );

        y =
            constrain(
                y,
                19,
                63
            );


        if (previousY >= 0)
        {
            display.drawLine(
                x - 1,
                previousY,
                x,
                y,
                SSD1306_WHITE
            );
        }

        previousY = y;
    }


    display.display();
}


void setup()
{
    Serial.begin(115200);


    pinMode(
        BEAT_PIN,
        INPUT
    );

    pinMode(
        BEAT_LED_PIN,
        OUTPUT
    );

    pinMode(
        LEADOFF_PIN,
        INPUT_PULLUP
    );

    digitalWrite(
        BEAT_LED_PIN,
        LOW
    );


    analogReadResolution(12);


    Wire.begin(
        OLED_SDA,
        OLED_SCL
    );

    Wire.setClock(400000);


    if (
        !display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_ADDRESS
        )
    )
    {
        while (true)
        {
            digitalWrite(
                BEAT_LED_PIN,
                HIGH
            );

            delay(100);

            digitalWrite(
                BEAT_LED_PIN,
                LOW
            );

            delay(100);
        }
    }


    for (
        int i = 0;
        i < WAVEFORM_SIZE;
        i++
    )
    {
        waveform[i] = 0.0f;
    }


    display.clearDisplay();

    display.setTextColor(
        SSD1306_WHITE
    );

    display.setTextSize(1);

    display.setCursor(0, 4);
    display.println("WEARABLE ECG BELT");

    display.setCursor(0, 18);
    display.println("CUSTOM ECG AFE");

    display.setCursor(0, 32);
    display.println("LEAD II  |  72 BPM");

    display.setCursor(0, 46);
    display.println("500 Hz");

    display.display();

    delay(1000);


    lastSampleTime =
        micros();

    lastOLEDTime =
        millis();
}


void loop()
{
    unsigned long now =
        micros();


    if (
        now -
        lastSampleTime
        <
        SAMPLE_US
    )
    {
        return;
    }


    lastSampleTime +=
        SAMPLE_US;


    bool leadOff =
        (
            digitalRead(
                LEADOFF_PIN
            ) == LOW
        );


    /* Always acquire the AFE output */
    ecg_mV =
        readECG();


    /* Beat handling */
    if (!leadOff)
    {
        processBeat();
    }
    else
    {
        digitalWrite(
            BEAT_LED_PIN,
            LOW
        );

        beatLEDActive = false;
        lastBeatTime = 0;
        hrValid = false;
    }


    /* Store waveform */
    waveform[
        waveformIndex
    ] =
        ecg_mV;

    waveformIndex++;

    if (
        waveformIndex >=
        WAVEFORM_SIZE
    )
    {
        waveformIndex = 0;
    }


    /* --------------------------------------------------------
     * SERIAL
     *
     * One numeric value only.
     * 100 samples/sec.
     * -------------------------------------------------------- */
    static uint8_t serialDivider = 0;

    serialDivider++;

    if (serialDivider >= 5)
    {
        serialDivider = 0;

        Serial.println(
            ecg_mV,
            2
        );
    }


    /* OLED ~20 FPS */
    unsigned long currentMillis =
        millis();

    if (
        currentMillis -
        lastOLEDTime
        >=
        50
    )
    {
        lastOLEDTime =
            currentMillis;

        drawOLED(
            leadOff
        );
    }
}