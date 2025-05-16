const byte encoder1PinA = 2;   // Interrupt-capable pins for encoder 1
const byte encoder1PinB = 3;
const byte encoder2PinA = 11;  // Interrupt-capable pins for encoder 2
const byte encoder2PinB = 12;

volatile long encoder1Count = 0;
volatile bool directionForward1 = true;
volatile long encoder2Count = 0;
volatile bool directionForward2 = true;

unsigned long prevTime = 0;

const int ppr = 1024;                   // Pulses per revolution
const float wheelRadius = 0.1016;        // Wheel radius in meters
const float wheelCircumference = 2 * PI * wheelRadius;

void setup() {
  Serial.begin(9600);
  // Configure encoder pins
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  
  // Attach interrupts for both encoders
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2, RISING);
}

void loop() {
  static long prevCount1 = 0;
  static long prevCount2 = 0;
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= 100) { // Update every 500ms
    noInterrupts();
    long count1 = encoder1Count;
    bool dir1 = directionForward1;
    long count2 = encoder2Count;
    bool dir2 = directionForward2;
    interrupts();

    long delta1 = count1 - prevCount1;
    long delta2 = count2 - prevCount2;
    prevCount1 = count1;
    prevCount2 = count2;

    // Calculate RPM for each encoder
    float rpm1 = (delta1 * 60.0) / (ppr * 0.5);
    float rpm2 = (delta2 * 60.0) / (ppr * 0.5);

    // Calculate displacement for each wheel
    float displacement1 = (float)count1 / ppr * wheelCircumference;
    float displacement2 = (float)count2 / ppr * wheelCircumference;

    // Print data in a parseable format
    Serial.print("E1_RPM:");
    Serial.print(rpm1, 2);
    Serial.print(",E1_DIR:");
    Serial.print(dir1 ? "F" : "B");
    Serial.print(",E1_DISP:");
    Serial.print(displacement1, 3);
    Serial.print(",E2_RPM:");
    Serial.print(rpm2, 2);
    Serial.print(",E2_DIR:");
    Serial.print(dir2 ? "F" : "B");
    Serial.print(",E2_DISP:");
    Serial.println(displacement2, 3);

    prevTime = currentTime;
  }
}

// Interrupt handlers for each encoder
void handleEncoder1() {
  bool bState = digitalRead(encoder1PinB);
  if (bState == LOW) {
    encoder1Count++;
    directionForward1 = true;
  } else {
    encoder1Count--;
    directionForward1 = false;
  }
}

void handleEncoder2() {
  bool bState = digitalRead(encoder2PinB);
  if (bState == LOW) {
    encoder2Count++;
    directionForward2 = true;
  } else {
    encoder2Count--;
    directionForward2 = false;
  }
}