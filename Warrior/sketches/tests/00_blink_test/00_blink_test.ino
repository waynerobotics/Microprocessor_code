#include <Arduino.h>

String serialLine = "";

#if defined(ARDUINO_NANO_ESP32)
const int redPin = LED_RED;
const int greenPin = LED_GREEN;
const int bluePin = LED_BLUE;
bool isInverted = true;
#else
const int redPin = 3;
const int greenPin = 5;
const int bluePin = 6;
bool isInverted = false;
#endif

void setup()
{
  Serial.begin(115200);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  allOff();

  Serial.println("00_blink_test ready");
}

void loop()
{
  readSerialInput();
}

void readSerialInput()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (serialLine.length() > 0)
      {
        handleSerialMessage(serialLine.c_str());
        serialLine = "";
      }
    }
    else
    {
      serialLine += c;
    }
  }
}

void handleSerialMessage(const char *message)
{
  if (strcmp(message, "WHO") == 0)
  {
    Serial.println("00_base");
  }
  else if (strcmp(message, "PING") == 0)
  {
    Serial.println("PONG");
  }
  else if (strncmp(message, "LED,", 4) == 0)
  {
    processInput(message[4]);
    Serial.println("LED");
  }
  else if (strlen(message) == 1)
  {
    processInput(message[0]);
    Serial.println("LED");
  }
  else
  {
    Serial.println("unknown_message");
  }
}

void processInput(char c)
{
  switch (c)
  {
  case 'r':
  case 'R':
    setPin(redPin, r);
    break;

  case 'g':
  case 'G':
    setPin(greenPin, g);
    break;

  case 'b':
  case 'B':
    setPin(bluePin, b);
    break;

  case 'w':
  case 'W':
    setColor(true, true, true);
    break;

  case 'o':
  case 'O':
    allOff();
    break;

  default:
    Serial.println("bad_led_command");
    break;
  }
}

void setPin(int pin, bool state)
{
  if (isInverted)
  {
    digitalWrite(pin, state ? LOW : HIGH);
  }
  else
  {
    digitalWrite(pin, state ? HIGH : LOW);
  }
}

void setColor(bool r, bool g, bool b)
{
  setPin(redPin, r);
  setPin(greenPin, g);
  setPin(bluePin, b);
}

void allOff()
{
  setColor(false, false, false);
}