#include <Arduino.h>
#include <LiquidCrystal.h>
#include <RadioLink.h>
#include <SerialProtocol.h>
#include <FanControl.h>
#include <IRremote.hpp>

// =====================================================================
// 00_base — base station
// Inputs:  LCD Keypad Shield (Shield)  and/or  RadioLink SBUS (Controller)
// Outputs: USB serial → Python bridge → 02/03/04_swerve
//
// Protocol: <MOT,<target>,<spark>,<flipsky>>
//   target  = "02_swerve" | "03_swerve" | "04_swerve"
//   spark   = -100..100
//   flipsky = -100..100
// One message is emitted per enabled target every SEND_INTERVAL_MS.
// Each swerve filters by its own DEVICE_NAME and ignores other targets.
//
// Toggle each target with SELECT (Shield) or buttonVRB (Controller).
// Cycle order: ALL → 02 only → 03 only → 04 only → ALL ...
//
// Hardware notes:
//   - Shield uses pins 4, 5, 6, 7, 8, 9 (LCD) and A0 (button divider).
//   - RadioLink reads SBUS on Serial1; specify the RX pin for ESP32.
//     On AVR (Mega), Serial1 RX is fixed (pin 19) and SBUS needs a
//     hardware inverter — set Controller=false if that isn't wired.
// =====================================================================

const char* DEVICE_NAME = "00_base";

// === Feature toggles =================================================
const bool Shield      = true;   // LCD Keypad Shield as input
const bool Controller  = false;  // RadioLink SBUS as input (ESP32 platforms)
const bool Fans        = false;   // NeoPixel fan-ring animation
const bool FairyLights = false;   // IR-toggled fairy light output

// === Per-swerve output toggles (default: all enabled) ================
bool swerve_02 = true;
bool swerve_03 = true;
bool swerve_04 = true;

// === Hardware =========================================================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // RS, Enable, D4, D5, D6, D7

const uint8_t SBUS_RX_PIN = 12;
// RadioLink     radio(Serial1, SBUS_RX_PIN);
SerialProtocol serialProtocol(DEVICE_NAME);

// NeoPixel fan rings — pin 10 stays clear of the LCD shield (4-9) and A0.
const uint8_t FAN_LED_PIN  = 10;
const uint8_t FAN_COUNT    = 3;
const uint8_t LEDS_PER_FAN = 20;
FanControl fans(FAN_LED_PIN, FAN_COUNT, LEDS_PER_FAN);

// Fairy lights: an IR receiver toggles a single output pin (drive a MOSFET
// or relay). Power button on the IR remote toggles the lights on/off.
const uint8_t IR_PIN              = 2;
const uint8_t FAIRY_LIGHT_PIN     = 11;
const uint32_t IR_POWER_BUTTON    = 0xBF40FF00;
bool fairyLightState = false;

// === Command state ====================================================
int currentSpark   = 0;
int currentFlipsky = 0;

#define CMD_MIN  -100
#define CMD_MAX   100
#define CMD_STEP  1
#define SEND_INTERVAL_MS    50   // 20 Hz output
#define BUTTON_INTERVAL_MS 100   // shield button rate-limit
#define LCD_INTERVAL_MS    100   // shield LCD refresh

// === Forward declarations ============================================
void handleShieldInput();
void handleControllerInput();
void cycleTargets();
void sendMotorMessages();
void sendMotorMessage(const char* target, int spark, int flipsky);
void updateLcd();
String readShieldButton();
void handleSerialMessage(const char* message);
void fairyLightSetup();
void fairyLightLoop();

// =====================================================================
void setup()
{
    serialProtocol.begin(115200);

    if (Shield) {
        lcd.begin(16, 2);
        lcd.setCursor(0, 0);
        lcd.print("Warrior 00_base");
        lcd.setCursor(0, 1);
        lcd.print("Ready");
    }

    if (Controller) {
        // radio.begin();
        // // Per-stick calibration (raw min/center/max measured on the radio)
        // radio.setChannelCalibration(1,  200,  822, 1598);  // CH2 elevator -> spark
        // radio.setChannelCalibration(3,  314, 1084, 1800);  // CH4 rudder   -> flipsky
    }

    if (Fans) {
        fans.begin();
    }

    if (FairyLights) {
        fairyLightSetup();
    }
}

void loop()
{
    if (Shield)     handleShieldInput();
    if (Controller) handleControllerInput();

    static uint32_t lastSendMs = 0;
    if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs = millis();
        sendMotorMessages();
    }

    if (Shield) {
        static uint32_t lastLcdMs = 0;
        if (millis() - lastLcdMs >= LCD_INTERVAL_MS) {
            lastLcdMs = millis();
            updateLcd();
        }
    }

    if (Fans) {
        fans.updateAnimation();
    }

    if (FairyLights) {
        fairyLightLoop();
    }

    serialProtocol.update();
    if (serialProtocol.hasMessage()) {
        handleSerialMessage(serialProtocol.getMessage());
        serialProtocol.clearMessage();
    }
}

// =====================================================================
// Shield input — buttons step the spark/flipsky values by CMD_STEP.
// SELECT cycles the active output targets.
// =====================================================================
void handleShieldInput()
{
    static uint32_t lastButtonMs = 0;
    if (millis() - lastButtonMs < BUTTON_INTERVAL_MS) return;

    String button = readShieldButton();
    if (button == "NONE") return;

    lastButtonMs = millis();

    if      (button == "UP")    currentSpark   = constrain(currentSpark   + CMD_STEP, CMD_MIN, CMD_MAX);
    else if (button == "DOWN")  currentSpark   = constrain(currentSpark   - CMD_STEP, CMD_MIN, CMD_MAX);
    else if (button == "RIGHT") currentFlipsky = constrain(currentFlipsky + CMD_STEP, CMD_MIN, CMD_MAX);
    else if (button == "LEFT")  currentFlipsky = constrain(currentFlipsky - CMD_STEP, CMD_MIN, CMD_MAX);
    else if (button == "SELECT") cycleTargets();
}

// =====================================================================
// Controller input — radio overwrites currentSpark/Flipsky every loop.
// On invalid frame, zeroes them so the watchdog on each swerve fires.
// buttonVRB rising-edge cycles output targets.
// =====================================================================
void handleControllerInput()
{
    // radio.update();
    // RadioLink::ControllerState state = radio.getState();

    // if (state.valid) {
    //     currentSpark   = state.elevator;
    //     currentFlipsky = state.rudder;
    // } else {
    //     currentSpark   = 0;
    //     currentFlipsky = 0;
    // }

    // static bool prevButtonVRB = false;
    // if (state.valid && state.buttonVRB && !prevButtonVRB) {
    //     cycleTargets();
    // }
    // prevButtonVRB = state.valid && state.buttonVRB;
}

// =====================================================================
// Cycle target enable mask: ALL → 02 → 03 → 04 → ALL ...
// =====================================================================
void cycleTargets()
{
    static int mode = 0;          // 0=ALL, 1=02, 2=03, 3=04
    mode = (mode + 1) % 4;
    swerve_02 = (mode == 0 || mode == 1);
    swerve_03 = (mode == 0 || mode == 2);
    swerve_04 = (mode == 0 || mode == 3);
}

// =====================================================================
// Output: emit one MOT message per enabled target.
// =====================================================================
void sendMotorMessages()
{
    if (swerve_02) sendMotorMessage("02_swerve", currentSpark, currentFlipsky);
    if (swerve_03) sendMotorMessage("03_swerve", currentSpark, currentFlipsky);
    if (swerve_04) sendMotorMessage("04_swerve", currentSpark, currentFlipsky);
}

void sendMotorMessage(const char* target, int spark, int flipsky)
{
    Serial.print("<MOT,");
    Serial.print(target);
    Serial.print(",");
    Serial.print(spark);
    Serial.print(",");
    Serial.print(flipsky);
    Serial.println(">");
}

// =====================================================================
// LCD: top line shows enabled targets, bottom shows current values.
// =====================================================================
void updateLcd()
{
    lcd.setCursor(0, 0);
    lcd.print(swerve_02 ? "[02]" : "[--]");
    lcd.print(swerve_03 ? "[03]" : "[--]");
    lcd.print(swerve_04 ? "[04]" : "[--]");
    lcd.print("    ");

    lcd.setCursor(0, 1);
    lcd.print("S:");
    lcd.print(currentSpark);
    lcd.print(" F:");
    lcd.print(currentFlipsky);
    lcd.print("        ");
}

String readShieldButton()
{
    int x = analogRead(A0);
    if (x < 60)   return "RIGHT";
    if (x < 200)  return "UP";
    if (x < 400)  return "DOWN";
    if (x < 600)  return "LEFT";
    if (x < 800)  return "SELECT";
    return "NONE";
}

// =====================================================================
// SerialProtocol: respond to WHO and PING. Other messages are ignored
// (unknown_message error). MOT is outbound-only from this device.
// =====================================================================
void handleSerialMessage(const char* message)
{
    if (strcmp(message, "WHO") == 0) {
        serialProtocol.sendDeviceName();
    } else if (strcmp(message, "PING") == 0) {
        serialProtocol.sendAck("PONG");
    } else {
        serialProtocol.sendError("unknown_message");
    }
}

// =====================================================================
// Fairy lights — IR remote toggles an output pin on/off.
// Drive the pin into a MOSFET or relay; don't pull mains light strings
// directly off an Arduino GPIO.
// =====================================================================
void fairyLightSetup()
{
    pinMode(FAIRY_LIGHT_PIN, OUTPUT);
    digitalWrite(FAIRY_LIGHT_PIN, LOW);
    IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

void fairyLightLoop()
{
    if (!IrReceiver.decode()) return;

    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    if (code == IR_POWER_BUTTON) {
        fairyLightState = !fairyLightState;
        digitalWrite(FAIRY_LIGHT_PIN, fairyLightState ? HIGH : LOW);
    }

    IrReceiver.resume();
}
