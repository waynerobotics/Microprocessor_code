#include <Arduino.h>
#include <SerialProtocol.h>

// =====================================================================
// 02_test — mock swerve client for reconnection / robustness testing
//
// Behaves on the wire exactly like 01_swerve (responds to WHO, PING, MOT)
// but drives no motors and uses no GPIO beyond an LED status indicator.
// Adds two diagnostic messages so a host script can verify the link:
//
//   <STATS>  →  <STATS,name,uptimeMs,motForMe,motForOther,parseErr,unknown,
//                       msSinceLast,rateHz,lastSpark,lastFlipsky>
//   <RESET>  →  zero counters, reply <ACK,RESET>
//
// Counters of interest:
//   motForMe       MOT messages whose target == DEVICE_NAME
//   motForOther    MOT messages addressed to a DIFFERENT swerve  (routing bug!)
//   parseErrors    malformed MOT messages
//   unknownMessages anything else
//
// Visual health (so you can yank cables and watch):
//   Nano ESP32: RGB onboard LED
//     green  = MOT seen in last 250 ms
//     yellow = 250..1000 ms since last MOT
//     red    = >1000 ms since last MOT
//     purple sticky ~2 s = saw motForOther or parseError (bad)
//     blue flash         = a message just arrived
//   Other boards (Mega/Uno): single LED_BUILTIN
//     solid on    = fresh
//     slow blink  = stale (>250 ms)
//     fast blink  = error sticky (~2 s after motForOther / parseError)
//
// IMPORTANT: stays SILENT on the wire unless asked. The serial_bridge does
// not read swerve responses, so unsolicited prints would fill the host's
// input buffer over time. STATS replies only fire on demand.
// =====================================================================

// === Identity (one swap) =============================================
// const char* DEVICE_NAME = "02_swerve";
// const char* DEVICE_NAME = "03_swerve";
const char* DEVICE_NAME = "04_swerve";

// All swerve names this mock recognizes (used to classify "for me" vs.
// "for someone else" so we can spot routing bugs).
const char* KNOWN_SWERVES[] = { "02_swerve", "03_swerve", "04_swerve" };
const uint8_t KNOWN_SWERVE_COUNT = sizeof(KNOWN_SWERVES) / sizeof(KNOWN_SWERVES[0]);

SerialProtocol serialProtocol(DEVICE_NAME);

// === Counters ========================================================
uint32_t motForMe        = 0;
uint32_t motForOther     = 0;
uint32_t parseErrors     = 0;
uint32_t unknownMessages = 0;

int  lastSpark   = 0;
int  lastFlipsky = 0;

uint32_t lastMotMs   = 0;   // millis() of most recent valid MOT for me
uint32_t lastErrorMs = 0;   // millis() of most recent routing/parse error

// Sliding 1-second rate window: ring of arrival timestamps for motForMe.
const uint8_t RATE_WINDOW = 32;
uint32_t rateRing[RATE_WINDOW] = {0};
uint8_t  rateHead = 0;

// === LED setup =======================================================
#if defined(ARDUINO_NANO_ESP32)
  const int LED_R_PIN = LED_RED;
  const int LED_G_PIN = LED_GREEN;
  const int LED_B_PIN = LED_BLUE;
  const bool LED_INVERTED = true;   // Nano ESP32: LOW = on
  #define HAS_RGB_LED 1
#else
  #define HAS_RGB_LED 0
#endif

const uint32_t STALE_FRESH_MS  = 250;
const uint32_t STALE_LONG_MS   = 1000;
const uint32_t ERROR_STICKY_MS = 2000;
const uint32_t BLUE_FLASH_MS   = 60;

uint32_t lastAnyMessageMs = 0;  // for the blue flash

// === Forward declarations ============================================
void handleSerialMessage(const char* message);
void handleMotorMessage(const char* message);
void handleStatsRequest();
void handleResetRequest();
void recordMotForMe(int spark, int flipsky);
uint32_t computeRateHz();
bool isKnownSwerve(const char* name);
void updateLed();
void setRgb(bool r, bool g, bool b);
void writeRgbPin(int pin, bool on);

// =====================================================================
void setup()
{
    serialProtocol.begin(115200);

#if HAS_RGB_LED
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    setRgb(false, false, false);
#else
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
#endif
}

void loop()
{
    serialProtocol.update();

    if (serialProtocol.hasMessage()) {
        lastAnyMessageMs = millis();
        handleSerialMessage(serialProtocol.getMessage());
        serialProtocol.clearMessage();
    }

    updateLed();
}

// =====================================================================
void handleSerialMessage(const char* message)
{
    if (strcmp(message, "WHO") == 0) {
        serialProtocol.sendDeviceName();
    } else if (strcmp(message, "PING") == 0) {
        serialProtocol.sendAck("PONG");
    } else if (strcmp(message, "STATS") == 0) {
        handleStatsRequest();
    } else if (strcmp(message, "RESET") == 0) {
        handleResetRequest();
    } else if (strncmp(message, "MOT,", 4) == 0) {
        handleMotorMessage(message);
    } else {
        unknownMessages++;
        // No reply — would fill bridge's unread buffer.
    }
}

void handleMotorMessage(const char* message)
{
    char target[24] = {0};
    int spark = 0, flipsky = 0;
    if (sscanf(message, "MOT,%23[^,],%d,%d", target, &spark, &flipsky) != 3) {
        parseErrors++;
        lastErrorMs = millis();
        return;
    }

    if (strcmp(target, DEVICE_NAME) == 0) {
        recordMotForMe(spark, flipsky);
    } else if (isKnownSwerve(target)) {
        // Addressed to another known swerve but landed on us — routing bug.
        motForOther++;
        lastErrorMs = millis();
    } else {
        // Unknown target name; treat as a parse-level anomaly rather than
        // silently ignoring, so the test driver can see it.
        parseErrors++;
        lastErrorMs = millis();
    }
    // No ACK — same constraint as the real swerve.
}

void recordMotForMe(int spark, int flipsky)
{
    motForMe++;
    lastSpark   = spark;
    lastFlipsky = flipsky;
    lastMotMs   = millis();

    rateRing[rateHead] = lastMotMs;
    rateHead = (rateHead + 1) % RATE_WINDOW;
}

uint32_t computeRateHz()
{
    // Count entries in rateRing within the last 1000 ms.
    uint32_t now = millis();
    uint32_t count = 0;
    for (uint8_t i = 0; i < RATE_WINDOW; i++) {
        uint32_t t = rateRing[i];
        if (t != 0 && now - t <= 1000) count++;
    }
    return count;  // already messages-per-second
}

bool isKnownSwerve(const char* name)
{
    for (uint8_t i = 0; i < KNOWN_SWERVE_COUNT; i++) {
        if (strcmp(name, KNOWN_SWERVES[i]) == 0) return true;
    }
    return false;
}

// =====================================================================
void handleStatsRequest()
{
    uint32_t now = millis();
    uint32_t msSinceLast = (lastMotMs == 0) ? 0xFFFFFFFFul : (now - lastMotMs);

    char buf[160];
    snprintf(buf, sizeof(buf),
             "<STATS,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d>",
             DEVICE_NAME,
             (unsigned long)now,
             (unsigned long)motForMe,
             (unsigned long)motForOther,
             (unsigned long)parseErrors,
             (unsigned long)unknownMessages,
             (unsigned long)msSinceLast,
             (unsigned long)computeRateHz(),
             lastSpark,
             lastFlipsky);
    Serial.println(buf);
}

void handleResetRequest()
{
    motForMe = motForOther = parseErrors = unknownMessages = 0;
    lastSpark = lastFlipsky = 0;
    lastMotMs = 0;
    lastErrorMs = 0;
    for (uint8_t i = 0; i < RATE_WINDOW; i++) rateRing[i] = 0;
    rateHead = 0;
    serialProtocol.sendAck("RESET");
}

// =====================================================================
void updateLed()
{
    uint32_t now = millis();
    bool errorSticky = (lastErrorMs != 0) && (now - lastErrorMs < ERROR_STICKY_MS);
    bool fresh       = (lastMotMs   != 0) && (now - lastMotMs   < STALE_FRESH_MS);
    bool stale       = (lastMotMs   != 0) && (now - lastMotMs   < STALE_LONG_MS);

#if HAS_RGB_LED
    bool blueFlash = (now - lastAnyMessageMs < BLUE_FLASH_MS);

    if (errorSticky) {
        // Purple = red + blue
        setRgb(true, false, true);
    } else if (blueFlash) {
        setRgb(false, false, true);
    } else if (fresh) {
        setRgb(false, true, false);
    } else if (stale) {
        // Yellow = red + green
        setRgb(true, true, false);
    } else {
        setRgb(true, false, false);
    }
#else
    // Single LED patterns.
    if (errorSticky) {
        // Fast blink at ~10 Hz
        digitalWrite(LED_BUILTIN, ((now / 50) & 1) ? HIGH : LOW);
    } else if (fresh) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        // Slow blink at ~1 Hz when stale or never-seen
        digitalWrite(LED_BUILTIN, ((now / 500) & 1) ? HIGH : LOW);
    }
#endif
}

#if HAS_RGB_LED
void setRgb(bool r, bool g, bool b)
{
    writeRgbPin(LED_R_PIN, r);
    writeRgbPin(LED_G_PIN, g);
    writeRgbPin(LED_B_PIN, b);
}

void writeRgbPin(int pin, bool on)
{
    if (LED_INVERTED) digitalWrite(pin, on ? LOW : HIGH);
    else              digitalWrite(pin, on ? HIGH : LOW);
}
#endif
