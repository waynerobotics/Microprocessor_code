/* * Hardware Settings 
 */
bool isInverted = true; // Set to true for Nano ESP32, false for Uno/Standard

#if defined(ARDUINO_NANO_ESP32)
  const int redPin = LED_RED;
  const int greenPin = LED_GREEN;
  const int bluePin = LED_BLUE;
#else
  const int redPin = 3;   
  const int greenPin = 5;
  const int bluePin = 6;
#endif

void setup() {
  Serial.begin(115200);
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  allOff();
  Serial.println("System Ready. Send r, g, b, or o (Press Enter).");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Filter out 'Newline' or 'Carriage Return' so they don't reset the LED
    if (cmd == '\n' || cmd == '\r') {
      return; 
    }

    processInput(cmd);
  }
}

// --- Functional Logic ---

void processInput(char c) {
  switch (c) {
    case 'r': 
      setColor(true, false, false);  
      Serial.println("State: RED");   
      break;
    case 'g': 
      setColor(false, true, false);  
      Serial.println("State: GREEN"); 
      break;
    case 'b': 
      setColor(false, false, true);  
      Serial.println("State: BLUE");  
      break;
    case 'o': 
      allOff();
      Serial.println("State: OFF"); 
      break;
    default:
      // Ignore unknown characters without changing the LED state
      break;
  }
}

void setPin(int pin, bool state) {
  if (isInverted) {
    digitalWrite(pin, state ? LOW : HIGH);
  } else {
    digitalWrite(pin, state ? HIGH : LOW);
  }
}

void setColor(bool r, bool g, bool b) {
  setPin(redPin, r);
  setPin(greenPin, g);
  setPin(bluePin, b);
}

void allOff() {
  setColor(false, false, false);
}