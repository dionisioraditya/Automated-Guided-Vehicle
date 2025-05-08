float voltage;

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL); // use (INTERNAL1V1) for a Mega
}

void loop() {
  voltage = analogRead(A0) * 0.01660 ; // calibrate by changing the last digit(s) of 0.0166
  //voltage -= 16.98;
  // print
  Serial.print("The supply is ");
  Serial.print(voltage);
  Serial.println(" volt");
  delay(1000); // remove when combine with other code
}