int const MIN_ANALOG = 124;
int const MAX_ANALOG = 840;

void setup() {
  Serial.begin(2000000);
}

void loop() {
  int in = analogRead(0);
  in = (in - MIN_ANALOG) * (1<<10 / (MAX_ANALOG - MIN_ANALOG));
  in /= 1<<2;
  Serial.write((char) in);
}
