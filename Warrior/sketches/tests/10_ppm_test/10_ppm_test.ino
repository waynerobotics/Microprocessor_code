#include <Arduino.h>
#include <RadioLink.h>
#include <MotorControl.h>

// Running on Arduino Nano ESP32
// Reads SBUS from a RadioLink R8EF (or compatible) on Serial1, maps the
// elevator/rudder sticks to -100..100 via RadioLink (with deadband and
// per-channel calibration), and drives D6/D7 ESC outputs via MotorControl.
//
// IMPORTANT: use raw GPIO numbers, not D6/D7 aliases — ESP32Servo misbehaves with the aliases

const uint8_t SBUS_RX_PIN  = 12;
const uint8_t SPARK_GPIO   = 9;   // = D6  (SparkMax steering ESC)
const uint8_t FLIPSKY_GPIO = 10;  // = D7  (Flipsky drive ESC)

RadioLink    radio(Serial1, SBUS_RX_PIN);
MotorControl motors(SPARK_GPIO, FLIPSKY_GPIO);

// --- Forward declarations ---
void radioToMotors();

void setup()
{
    Serial.begin(115200);
    radio.begin();
    motors.begin();

    // Calibration values measured from the actual transmitter sticks.
    // (channel index, raw min, raw center, raw max)
    radio.setChannelCalibration(1,  200,  822, 1598);  // CH2 elevator -> spark
    radio.setChannelCalibration(3,  314, 1084, 1800);  // CH4 rudder   -> flipsky

    Serial.println("System Online: SparkMax (D6) + Flipsky (D7)");
}

void loop()
{
    radioToMotors();
    motors.update();

    // Status print at 10 Hz
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        RadioLink::ControllerState state = radio.getState();
        Serial.print(state.valid ? "[OK] " : "[INVALID] ");
        Serial.print("elev=");    Serial.print(state.elevator);
        Serial.print(" -> spark="); Serial.print(motors.getSparkMicroseconds());
        Serial.print("us | rud="); Serial.print(state.rudder);
        Serial.print(" -> flipsky="); Serial.print(motors.getFlipskyMicroseconds());
        Serial.println("us");
    }
}

// Reads the latest SBUS frame, maps elevator/rudder to -100..100 via RadioLink
// (deadband + per-channel calibration applied internally), and forwards to
// MotorControl. If the radio frame is invalid (failsafe, frame loss, stale),
// no command is sent — MotorControl's watchdog will return motors to neutral.
void radioToMotors()
{
    radio.update();
    RadioLink::ControllerState state = radio.getState();
    if (state.valid) {
        motors.setCommands(state.elevator, state.rudder);
    }
}
