void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CLEARDATA"); // Clear old data
  Serial.println("LABEL,Time,SensorValue"); // Column headers
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(A0);
  Serial.print("DATA,TIME,"); // TIME = Excel's current time
  Serial.println(sensorValue);
  delay(1000); // 1 second interval
}
