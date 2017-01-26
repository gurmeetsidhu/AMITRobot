int sensorPin = 1; //analog pin 0

void setup(){
  Serial.begin(9600);
}

void loop(){
  int val = analogRead(sensorPin);
  Serial.println(val);
}
