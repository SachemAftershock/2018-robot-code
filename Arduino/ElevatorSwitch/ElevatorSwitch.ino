int RELAY_ONE_OUT = 7;
int RELAY_THREE_OUT = 5;
int OVERRIDE = 0;

void setup() {
  pinMode(RELAY_ONE_OUT  , OUTPUT       );
  pinMode(RELAY_THREE_OUT, OUTPUT       );
  pinMode(OVERRIDE,        INPUT );
}

void loop() {
  boolean isOverride = digitalRead(OVERRIDE);
  // Debounce input
  delay(10);
  isOverride &= digitalRead(OVERRIDE);
  digitalWrite(RELAY_ONE_OUT,   isOverride);
  digitalWrite(RELAY_THREE_OUT, isOverride);
}
