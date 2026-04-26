#include <IRremote.hpp>

#define IR_PIN 12
const int lightPin = 11;

bool lightState = false;

//#define POWER_BUTTON 0xBA45FF00  // replace with your code
#define POWER_BUTTON 0xBF40FF00
void setup() {
  pinMode(lightPin, OUTPUT);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (IrReceiver.decode()) {

    unsigned long code = IrReceiver.decodedIRData.decodedRawData;

    if (code == POWER_BUTTON) {
      lightState = !lightState;  // toggle lights
    }

    digitalWrite(lightPin, lightState ? HIGH : LOW);

    IrReceiver.resume();
  }
}

/*#include <IRremote.hpp>

#define IR_PIN 2

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.print("Code: ");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.resume(); // ready for next signal
  }
}
*/
