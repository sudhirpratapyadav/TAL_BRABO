void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
DDRC = B00000000;
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensor_val = PINC;
  Serial.println(sensor_val);
  delay(10);
}
