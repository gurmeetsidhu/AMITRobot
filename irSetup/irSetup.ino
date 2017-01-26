//#define motor1 7
//#define motor2 8
//#define echo 9
#define trig 7

void setup()
{

  Serial.begin(9600);
  //pinMode(motor1, OUTPUT);
  //pinMode(motor2, OUTPUT);
  pinMode(trig, INPUT);
  //pinMode(echo, INPUT);

}

void loop()
{

  float duration, volts, distance;
  //digitalWrite(trig, LOW);
  //delayMicroseconds(2);
  //digitalWrite(trig, HIGH);
  //delayMicroseconds(10);
  //digitalWrite(trig, LOW);

  duration = pulseIn(trig, HIGH);

  volts = (analogRead(1))*0.0048828125;
  distance = -(2.01423036*pow(volts,5)) + (19.44766216*pow(volts,4)) - (73.53682794*pow(volts,3)) + (138.47666662*pow(volts,2)) - (136.60286212*pow(volts,1)) + 66.31890266;
  Serial.print("Volts: ");
  Serial.println(volts);
  Serial.print(" Distance: ");
  Serial.println(distance);

}
