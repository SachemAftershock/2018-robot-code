void setup() {
  Serial.begin(2000000);
}

void loop() {
  Serial.write((char) (analogRead(0) / (1<<2)));
  delay(100);
}
