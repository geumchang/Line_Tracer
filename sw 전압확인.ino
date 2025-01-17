void setup() {
Serial.begin(115200);
}
void loop() {
int adcInput = analogRead(A0);
int voltage = map(adcInput, 0, 1023, 0, 5000);
Serial.println(voltage);
delay(1000);
}
