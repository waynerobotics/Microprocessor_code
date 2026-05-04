#include <LiquidCrystal.h>
#include <SerialProtocol.h>

const char* DEVICE_NAME = "00_base";

SerialProtocol serialProtocol(DEVICE_NAME);

// LCD Keypad Shield pins: RS, Enable, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Command state – normalized -100..100, 0 = neutral
int currentSpark   = 0;
int currentFlipsky = 0;

#define CMD_MIN  -100
#define CMD_MAX   100
#define CMD_STEP  1
#define SEND_INTERVAL_MS 50    // 20 Hz send rate
#define BUTTON_INTERVAL_MS 100 // step applied at most every 100 ms

void setup() {
  serialProtocol.begin(115200);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Warrior 00_base");
  lcd.setCursor(0, 1);
  lcd.print("Ready");
}

void loop() {
  // --- Button input (rate-limited to BUTTON_INTERVAL_MS) ---
  static uint32_t lastButtonMs = 0;
  if (millis() - lastButtonMs >= BUTTON_INTERVAL_MS) {
    String button = readButtonName();
    if (button != "NONE") {
      lastButtonMs = millis();
      if (button == "UP")    currentSpark   = constrain(currentSpark   + CMD_STEP, CMD_MIN, CMD_MAX);
      if (button == "DOWN")  currentSpark   = constrain(currentSpark   - CMD_STEP, CMD_MIN, CMD_MAX);
      if (button == "RIGHT") currentFlipsky = constrain(currentFlipsky + CMD_STEP, CMD_MIN, CMD_MAX);
      if (button == "LEFT")  currentFlipsky = constrain(currentFlipsky - CMD_STEP, CMD_MIN, CMD_MAX);
      if (button == "SELECT") { currentSpark = 0; currentFlipsky = 0; }
    }
  }

  // --- Send motor command to Python bridge at 20 Hz ---
  static uint32_t lastSendMs = 0;
  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    sendMotorMessage(currentSpark, currentFlipsky);
  }

  // --- Update LCD every 100 ms ---
  static uint32_t lastLcdMs = 0;
  if (millis() - lastLcdMs >= 100) {
    lastLcdMs = millis();
    lcd.setCursor(0, 0);
    lcd.print("SP:");
    lcd.print(currentSpark);
    lcd.print("            ");
    lcd.setCursor(0, 1);
    lcd.print("FL:");
    lcd.print(currentFlipsky);
    lcd.print("            ");
  }

  // --- Handle incoming serial messages (WHO, PING) ---
  serialProtocol.update();
  if (serialProtocol.hasMessage()) {
    handleSerialMessage(serialProtocol.getMessage());
    serialProtocol.clearMessage();
  }
}

void handleSerialMessage(const char* message) {
  if (strcmp(message, "WHO") == 0) {
    serialProtocol.sendDeviceName();
  } else if (strcmp(message, "PING") == 0) {
    serialProtocol.sendAck("PONG");
  } else {
    serialProtocol.sendError("unknown_message");
  }
}

void sendMotorMessage(int spark, int flipsky) {
  Serial.print("<MOT,");
  Serial.print(spark);
  Serial.print(",");
  Serial.print(flipsky);
  Serial.println(">");
}

String readButtonName() {
  int x = analogRead(A0);

  if (x < 60)   return "RIGHT";
  if (x < 200)  return "UP";
  if (x < 400)  return "DOWN";
  if (x < 600)  return "LEFT";
  if (x < 800)  return "SELECT";
  return "NONE";
}
