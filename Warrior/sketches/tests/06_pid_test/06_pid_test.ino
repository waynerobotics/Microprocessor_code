const int IN_PIN = D7;

void setup() {
  Serial.begin(115200);
  pinMode(IN_PIN, INPUT);

  Serial.println("Reading PWM on D7");
}

void loop() {
  unsigned long duration = pulseIn(IN_PIN, HIGH, 25000);

  Serial.print("Pulse: ");
  Serial.print(duration);
  Serial.println(" us");

  delay(200);
}