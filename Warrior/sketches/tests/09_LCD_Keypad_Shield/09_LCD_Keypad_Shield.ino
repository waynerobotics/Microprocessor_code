#include <LiquidCrystal.h>

// Pins for the Inland/DFRobot LCD Keypad Shield
// RS, Enable, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup() {
  lcd.begin(16, 2);              // Initialize 16x2 characters
  lcd.setCursor(0, 0);           // Top left
  lcd.print("Hello World!");
  lcd.setCursor(0, 1);           // Bottom left
  lcd.print("Press a Key...");
}

void loop() {
  lcd.setCursor(0, 1);           // Move cursor to bottom row
  int x = analogRead(0);         // Read the button value from A0

  // The shield uses a resistor ladder. Values might vary slightly by 
  // manufacturer, but these ranges are standard for Inland/DFRobot:
  if (x < 60) {
    lcd.print("Key: RIGHT     ");
  } else if (x < 200) {
    lcd.print("Key: UP        ");
  } else if (x < 400) {
    lcd.print("Key: DOWN      ");
  } else if (x < 600) {
    lcd.print("Key: LEFT      ");
  } else if (x < 800) {
    lcd.print("Key: SELECT    ");
  } else {
    lcd.print("Key: NONE      ");
  }
}