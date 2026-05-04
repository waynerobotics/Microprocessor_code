#include <Arduino.h>
#include <ESP32Servo.h>

// --- Configuration ---
#define SBUS_PIN 12
#define SPARK_PIN D6   // SparkMax ESC PWM output
#define FLIPSKY_PIN D7 // Flipsky ESC PWM output

#define SBUS_NUM_CHANNELS 8
#define DEADBAND 5

// --- Global Variables ---
uint16_t sbusChannels[SBUS_NUM_CHANNELS];
uint8_t sbusFrame[25];
uint8_t sbusFramePos = 0;

Servo spark;
Servo flipsky;

// --- Main Execution Loop ---
void loop()
{
    // Read SBUS first
    sbusRead();

    // Map and write PWM immediately
    int sparkPWM = mapWithDeadband(sbusChannels[1], 200, 822, 1598, DEADBAND);
    int flipskyPWM = mapWithDeadband(sbusChannels[3], 314, 1084, 1800, DEADBAND);

    spark.writeMicroseconds(sparkPWM);
    flipsky.writeMicroseconds(flipskyPWM);

    // Non-blocking serial print (every 100ms)
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();
        Serial.print("Spark (ELE): ");
        Serial.print(sparkPWM);
        Serial.print(" | Flipsky (RUD): ");
        Serial.println(flipskyPWM);
    }
}

// --- Helper Functions ---

void sbusRead()
{
    // Process at most one full frame worth of bytes per call to stay non-blocking
    // Flush all but the last 25 bytes so we always parse the freshest frame
    while (Serial1.available() > 25)
        Serial1.read();

    // Now read exactly up to 25 bytes
    while (Serial1.available())
    {
        uint8_t b = Serial1.read();
        if (sbusFramePos == 0 && b != 0x0F)
            continue;
        sbusFrame[sbusFramePos++] = b;

        if (sbusFramePos == 25)
        {
            if (sbusFrame[24] == 0x00)
            {
                sbusChannels[0] = ((sbusFrame[1] | sbusFrame[2] << 8) & 0x07FF);
                sbusChannels[1] = ((sbusFrame[2] >> 3 | sbusFrame[3] << 5) & 0x07FF);
                sbusChannels[2] = ((sbusFrame[3] >> 6 | sbusFrame[4] << 2 | sbusFrame[5] << 10) & 0x07FF);
                sbusChannels[3] = ((sbusFrame[5] >> 1 | sbusFrame[6] << 7) & 0x07FF);
                sbusChannels[4] = ((sbusFrame[6] >> 4 | sbusFrame[7] << 4) & 0x07FF);
                sbusChannels[5] = ((sbusFrame[7] >> 7 | sbusFrame[8] << 1 | sbusFrame[9] << 9) & 0x07FF);
                sbusChannels[6] = ((sbusFrame[9] >> 2 | sbusFrame[10] << 6) & 0x07FF);
                sbusChannels[7] = ((sbusFrame[10] >> 5 | sbusFrame[11] << 3) & 0x07FF);
            }
            sbusFramePos = 0;
        }
    }
}

int mapWithDeadband(int val, int minIn, int centerIn, int maxIn, int deadband)
{
    // If value is within the deadband range, return 1500 (neutral)
    if (val >= (centerIn - deadband) && val <= (centerIn + deadband))
    {
        return 1500;
    }

    // Piecewise mapping for accurate center-to-edge travel
    if (val < centerIn)
    {
        return map(val, minIn, centerIn, 1000, 1500);
    }
    else
    {
        return map(val, centerIn, maxIn, 1500, 2000);
    }
}

void setup()
{
    Serial.begin(115200);
    // Use 'true' for SBUS inversion on Pin 12
    Serial1.begin(100000, SERIAL_8E2, SBUS_PIN, -1, true);

    // Use GPIO numbers directly to avoid mapping errors
    const int sparkGPIO = 9;    // This is D6
    const int flipskyGPIO = 10; // This is D7

    spark.attach(sparkGPIO, 1000, 2000);
    flipsky.attach(flipskyGPIO, 1000, 2000);

    Serial.println("System Online: SparkMax (GPIO 9) & Flipsky (GPIO 10)");
}