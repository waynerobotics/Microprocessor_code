// 01_bit_banging.ino
//
// Standalone software SBUS reader. No libraries beyond core Arduino.
// Reads raw SBUS on a single pin via an external interrupt + busy-wait
// sampling, decodes the 8 main channels, and prints them to the Serial
// monitor at 115200 baud.
//
// Targets the Arduino Uno R3 (no hardware Serial1). Should also run on
// Mega/Leonardo if you wire to pin 2 or 3 (INT0/INT1).
//
// Hardware:
//   - Wire RC receiver SBUS output → Arduino digital pin 2 (INT0).
//     If you have a hardware inverter circuit between the receiver and
//     the Arduino, change SBUS_INVERTED to false.
//   - Common GND between receiver and Arduino.
//   - Receiver powered from 5V (or its own supply).
//
// Notes / known limitations:
//   - The ISR busy-waits ~110 µs per byte, blocking other interrupts.
//   - If anything else in the sketch disables interrupts for >5 µs
//     during a SBUS frame (NeoPixels, long ISRs, etc.) the frame will
//     fail validation. The next frame ~14 ms later usually recovers.
//   - Bit timing on a 16 MHz AVR is marginal because each loop pass is
//     slightly more than 10 µs. _delay_us(9) is used to compensate; if
//     channels read garbage, try _delay_us(10).
//   - Only pins 2 (INT0) and 3 (INT1) are supported; PCINT pins would
//     work too but require manual register setup not done here.
//
// SBUS protocol summary (for reference):
//   - 100000 baud, 8 data + even parity + 2 stop, INVERTED
//   - Idle line LOW (raw); start bit goes HIGH on a rising edge
//   - 25 bytes per frame: 0x0F header, 22 channel data bytes,
//     1 flags byte, 0x00 footer
//   - Frame interval ~14 ms

#include <util/delay.h>

// === Configuration ====================================================
const uint8_t SBUS_PIN      = 2;     // INT0 (Uno) — change to 3 for INT1
const bool    SBUS_INVERTED = true;  // true = raw SBUS line, false = hw inverted
const uint32_t PRINT_INTERVAL_MS = 100;

// === SBUS frame constants ============================================
const uint8_t SBUS_FRAME_SIZE   = 25;
const uint8_t SBUS_HEADER       = 0x0F;
const uint8_t SBUS_FOOTER       = 0x00;
const uint8_t SBUS_FLAG_FRAME_LOST = 0x04;
const uint8_t SBUS_FLAG_FAILSAFE   = 0x08;

// === ISR state =======================================================
volatile uint8_t  sbusBuf[SBUS_FRAME_SIZE];
volatile uint8_t  sbusBufIdx   = 0;
volatile bool     sbusFrameReady = false;
volatile bool     sbusInByte   = false;
volatile uint32_t sbusByteCount = 0;
volatile uint32_t sbusFrameCount = 0;

volatile uint8_t* sbusPortIn = nullptr;
uint8_t           sbusPortMask = 0;

// =====================================================================
// External-interrupt ISR. Hardware globally disables interrupts on entry,
// so we sample with deterministic timing. The ISR runs ~110 µs per byte.
// =====================================================================
void sbusISR()
{
    if (sbusInByte) return;
    sbusInByte = true;

    // Mid-bit-0 sample point is 15 µs after the start edge. ISR prologue
    // already burned ~2 µs, so wait ~13 µs more.
    _delay_us(13);

    uint8_t b = 0;
    uint8_t bitMask = 0x01;
    for (uint8_t i = 0; i < 8; i++) {
        bool lineHigh = (*sbusPortIn & sbusPortMask) != 0;
        // Logical bit value = line state (or its inverse if SBUS is inverted).
        bool bit = SBUS_INVERTED ? !lineHigh : lineHigh;
        if (bit) b |= bitMask;
        bitMask <<= 1;
        _delay_us(9);  // tuned slightly low to offset loop overhead
    }

    // Skip parity + first stop bit's mid-point (we don't need them)
    _delay_us(9);

    sbusByteCount++;

    // Frame assembly: hunt for header at byte 0, then collect 24 more.
    if (sbusBufIdx == 0 && b != SBUS_HEADER) {
        // discard until we see a header
    } else if (sbusBufIdx < SBUS_FRAME_SIZE) {
        sbusBuf[sbusBufIdx++] = b;
        if (sbusBufIdx == SBUS_FRAME_SIZE) {
            if (sbusBuf[24] == SBUS_FOOTER) {
                sbusFrameReady = true;
                sbusFrameCount++;
            }
            sbusBufIdx = 0;
        }
    }

    sbusInByte = false;
}

// =====================================================================
void setup()
{
    Serial.begin(115200);

    pinMode(SBUS_PIN, INPUT_PULLUP);
    sbusPortIn   = portInputRegister(digitalPinToPort(SBUS_PIN));
    sbusPortMask = digitalPinToBitMask(SBUS_PIN);

    int n = digitalPinToInterrupt(SBUS_PIN);
    if (n == NOT_AN_INTERRUPT) {
        Serial.print(F("ERROR: pin "));
        Serial.print(SBUS_PIN);
        Serial.println(F(" is not an interrupt pin (use 2 or 3)"));
        while (true) {}
    }

    // Inverted SBUS: start bit is a LOW->HIGH transition (rising)
    // After hardware inverter: start bit is HIGH->LOW (falling)
    attachInterrupt(n, sbusISR, SBUS_INVERTED ? RISING : FALLING);

    Serial.print(F("01_bit_banging — software SBUS reader on pin "));
    Serial.println(SBUS_PIN);
    Serial.print(F("Inverted: "));
    Serial.println(SBUS_INVERTED ? "true" : "false");
}

void loop()
{
    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs < PRINT_INTERVAL_MS) return;
    lastPrintMs = millis();

    // Snapshot ISR counters for diagnostics
    noInterrupts();
    uint32_t bytes  = sbusByteCount;
    uint32_t frames = sbusFrameCount;
    bool ready = sbusFrameReady;
    uint8_t f[SBUS_FRAME_SIZE];
    if (ready) {
        for (uint8_t i = 0; i < SBUS_FRAME_SIZE; i++) f[i] = sbusBuf[i];
        sbusFrameReady = false;
    }
    interrupts();

    if (!ready) {
        Serial.print(F("(no fresh frame)  bytes="));
        Serial.print(bytes);
        Serial.print(F("  frames="));
        Serial.println(frames);
        return;
    }

    // Decode the 8 main channels (11-bit each, packed)
    uint16_t ch[8];
    ch[0] = ((f[1]      | f[2] << 8)                    & 0x07FF);
    ch[1] = ((f[2] >> 3 | f[3] << 5)                    & 0x07FF);
    ch[2] = ((f[3] >> 6 | f[4] << 2 | f[5] << 10)       & 0x07FF);
    ch[3] = ((f[5] >> 1 | f[6] << 7)                    & 0x07FF);
    ch[4] = ((f[6] >> 4 | f[7] << 4)                    & 0x07FF);
    ch[5] = ((f[7] >> 7 | f[8] << 1 | f[9] << 9)        & 0x07FF);
    ch[6] = ((f[9] >> 2 | f[10] << 6)                   & 0x07FF);
    ch[7] = ((f[10] >> 5 | f[11] << 3)                  & 0x07FF);

    bool frameLost = (f[23] & SBUS_FLAG_FRAME_LOST) != 0;
    bool failsafe  = (f[23] & SBUS_FLAG_FAILSAFE)   != 0;

    Serial.print(F("CH "));
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print(ch[i]);
        Serial.print(' ');
    }
    if (frameLost) Serial.print(F(" [LOST]"));
    if (failsafe)  Serial.print(F(" [FAILSAFE]"));
    Serial.print(F("  bytes="));
    Serial.print(bytes);
    Serial.print(F(" frames="));
    Serial.println(frames);
}
