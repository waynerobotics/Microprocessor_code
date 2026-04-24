#include <LiquidCrystal.h>
#include <SerialProtocol.h>

const char* DEVICE_NAME = "09_lcd_keypad";
const bool USE_SERIAL_PROTOCOL = true;

SerialProtocol serialProtocol(DEVICE_NAME);

String serialLine = "";

// LCD Keypad Shield pins: RS, Enable, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

String lastButton = "NONE";
unsigned long lastButtonSendMs = 0;

void setup() {
  serialProtocol.begin(115200);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Warrior LCD");
  lcd.setCursor(0, 1);
  lcd.print("Serial Ready");

  Serial.println("09_LCD_Keypad_Shield ready");
}

void loop() {
  readSerialInput();
  updateButtons();
}

void readSerialInput() {
  if (USE_SERIAL_PROTOCOL) {
    serialProtocol.update();

    if (serialProtocol.hasMessage()) {
      handleSerialMessage(serialProtocol.getMessage());
      serialProtocol.clearMessage();
    }
  } else {
    while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (serialLine.length() > 0) {
          handleSerialMessage(serialLine.c_str());
          serialLine = "";
        }
      } else {
        serialLine += c;
      }
    }
  }
}

void handleSerialMessage(const char* message) {
  if (strcmp(message, "WHO") == 0) {
    serialProtocol.sendDeviceName();
  }
  else if (strcmp(message, "PING") == 0) {
    serialProtocol.sendAck("PONG");
  }
  else if (strncmp(message, "LCD,", 4) == 0) {
    handleLcdMessage(message);
  }
  else if (strcmp(message, "CLEAR") == 0) {
    lcd.clear();
    serialProtocol.sendAck("CLEAR");
  }
  else {
    serialProtocol.sendError("unknown_message");
  }
}

void handleLcdMessage(const char* message) {
  // Format:
  // LCD,row,text
  //
  // Examples:
  // <LCD,0,Hello>
  // <LCD,1,Warrior>

  int firstComma = findComma(message, 0);
  int secondComma = findComma(message, firstComma + 1);

  if (firstComma < 0 || secondComma < 0) {
    serialProtocol.sendError("bad_lcd_message");
    return;
  }

  int row = String(message).substring(firstComma + 1, secondComma).toInt();
  String text = String(message).substring(secondComma + 1);

  if (row < 0 || row > 1) {
    serialProtocol.sendError("bad_lcd_row");
    return;
  }

  lcd.setCursor(0, row);
  lcd.print("                ");
  lcd.setCursor(0, row);
  lcd.print(text.substring(0, 16));

  serialProtocol.sendAck("LCD");
}

void updateButtons() {
  String button = readButtonName();

  if (button != lastButton || millis() - lastButtonSendMs > 1000) {
    lastButton = button;
    lastButtonSendMs = millis();

    Serial.print("<BTN,");
    Serial.print(button);
    Serial.println(">");
  }
}

String readButtonName() {
  int x = analogRead(A0);

  if (x < 60) {
    return "RIGHT";
  } else if (x < 200) {
    return "UP";
  } else if (x < 400) {
    return "DOWN";
  } else if (x < 600) {
    return "LEFT";
  } else if (x < 800) {
    return "SELECT";
  } else {
    return "NONE";
  }
}

int findComma(const char* message, int startIndex) {
  for (int i = startIndex; message[i] != '\0'; i++) {
    if (message[i] == ',') {
      return i;
    }
  }

  return -1;
}