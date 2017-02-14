# AMITRobot
AMITRobot stands for Autonomous Mining Intervention Technology Robot.

# Initial Library Setup
Ensure that all libraries are placed within respective folders to ensure proper running of code. Note that the Pololu wheel encoder library goes under Arduino Folder>Hardware instead of Folder>Libraries
```
#include <PololuWheelEncoders.h>
#include <PID_v1.h>
#define trig 0
```

# Variable Decleration
Declare constants that will be used to modular turning function that uses encoder to ensure accurate turning of robot.
```
const float pi = 3.14;
cosnt int encoderUnitValue = 130;	//Encoder count before the wheel has moved 1cm. Value provided online and measured in tests. Can be adjusted to account for general slip.
```

Declare speed variables that will be used to control outputs to individual motors from PID, turning or stopping functions.
```
int speedL, speedR;
int speed=130;	//Maximum speed provided to stronger wheel. Can be adjusted here. Note changing this requires that PIDs be retuned (Later will use RPM so this is no longer needed)
float ratio;	//Used to handle deciding initial turning speed of inner wheel in encoder turning function
```

# Modular Function Decleration
Declare distance variables for each IR sensor and distance calculation function that is used throughout the code for calculating distance from wall. This function was calibrated and developed seperately using pySerial output with polynomial regression to develop a more accurate function than online versions.
```
float distanceL, distanceR, distanceC, distanceM;
float lastDistL, lastDistR, lastDistM, lastDistC;	//Used to store last IR values for numerous functions to compare again at the beginning of the loop
float distance(int analogInput) {	//returns IR sensor reading when provided port number.
  double volts = (analogRead(analogInput))*0.0048828125;	//Convert analog input to voltage
  double dist = -(2.01423036*pow(volts, 5)) + (19.44766216*pow(volts, 4)) - (73.53682794*pow(volts, 3)) + (138.47666662*pow(volts, 2)) - (136.60286212*pow(volts, 1)) + 66.31890266;
  return dist;	//return calculated distance above
}
```

Declare stop function that writes to both motor power outputs M1,M2 a value of zero indicating a hard stop for both wheels. Used for crash prevention and stopping for miner.
```
void motorFullStop() {	//No parameters needed simply call function
  analogWrite(6, 0);	//Right motor power = 0
  analogWrite(5, 0);	//Left motor power = 0
}
```

Deprecated turning function that used encoder and a PID to accurately and robustly execute turns based on input infrared values. Determined outer/inner turning radiuses and used a PID to ensure accurate execution. Will be revisited and probably heavily modified. Used for second coding prototype.
```
/*
//PID turningPID(&Input2, &Output2, &Setpoint2, Kpt, Kit, Kdt, DIRECT);	//Unique turning PID to handle variable setpoint (inner turning radius) to ensure turn is made and motor spinning accurately
void motorTankTurn(int dir, double rad) {	//Direction to turn and inner radius of left/right turn as measured by respective IR sensor 
    PololuWheelEncoders::getCountsAndResetM2();	//Reset encoder to zero for encoder measurement later on for turn execution
    PololuWheelEncoders::getCountsAndResetM1();
    double distRight, distLeft;	//Initialize variables that will store how much encoder has turned on each side.
    turningPID.SetTunings(Kpt, Kit, Kdt);	//set tuning setting for PID. Potential dual-stage PID application left open here
    turningPID.SetOutputLimits(-260,-125);	//sets max/min value that the PID can output to control the turn. Reasonable values selected so no under/oversteer.
    if (rad>7){	//if radius to great set minimum inner turning radius. Otherwise too wide of a turn will hit outer wall.
      rad=5.5;
    }
    double innerRad, outerRad, ratio;	//Values to define maximum turning required by the encoder for inner/outer wheel and ratio to use to control wheel.
//    Serial.print(" <- Inner Radius(cm) Outer => "); //Used to output what robot is thinking and current turning status
//    Serial.println(outerRad/encoderUnitValue);
    switch(dir){	//Execute different turning loops depending on indicated turn direction
      case 1: //Turn left
        innerRad = (rad+Setpoint)*pi/4*encoderUnitValue;	//Math to determine inner radius
        outerRad = (rad+Setpoint+20)*pi/4*encoderUnitValue;	//Math to determine outer radius. Note vehicle width is 10cm.
        ratio = outerRad/innerRad;	//Ratio of initial power outputs. This will be more effective when PID will control rotation not voltage output
        speedR=maxSpeed;	//Outer wheel speed aka right wheel.
        speedL=maxSpeed/ratio;	//Inner wheel speed aka left wheel.
        analogWrite(6, speedL);	//left motor speed
        analogWrite(5, speedR);	//right motor speed
        digitalWrite(7, LOW);	//left motor direction
        digitalWrite(4, LOW);	//right motor direction
        distRight = (-PololuWheelEncoders::getCountsM1());	//get count of how much right motor turned
        distLeft = (PololuWheelEncoders::getCountsM2());	//get count of how much left motor turned
        Setpoint2 = 0;	
        while((distRight<outerRad  ||  distLeft<innerRad)){	//while either outer or inner wheel has not turned its full amount do...
          Input2 = distLeft*ratio-distRight;	//calculate how much extra the left wheel has spun. Can be negative or positive.
          turningPID.Compute();	//Feed that into the PID library which uses input and provided Kp, Ki, Kd values to create Output2 variable.
          speedL=255+Output2;	//adjust left speed by applying output to full speed value.
          Serial.print(speedL);	//Print left/right/output speed for troubleshooting. Note right speed is always 255. Should be lowered for more accuracy.
          Serial.print(" <- Left Speed Right -> 255 Output: ");
          Serial.println(Output2);
          analogWrite(6, abs(speedL));	//left motor speed
          analogWrite(5, 255);	//right motor speed
          if(speedL<0){ digitalWrite(7, HIGH); }
          else        { digitalWrite(7, LOW);  }	//left motor direction. Use simple if statement to assign right high/low value. Low=forward, High=backward
          digitalWrite(4, LOW);	//right motor direction. Always forward, always full power.
          distRight = (-PololuWheelEncoders::getCountsM1());	//calculate distance again now, to see if motor has turned enough to exit while loop.
          distLeft = (PololuWheelEncoders::getCountsM2());
        }
      case -1: //turn right. Similiar to above but polarity is flipped.
        analogWrite(6, 200); //left motor speed
        analogWrite(5, 220); //right motor speed
        digitalWrite(7, LOW); //left motor directiom
        digitalWrite(4, LOW); //right motor direction
        delay(300);
        innerRad = (rad+Setpoint)*pi/4*encoderUnitValue;
        outerRad = (rad+Setpoint+20)*pi/4*encoderUnitValue;
        ratio = outerRad/innerRad;
        speedL=maxSpeed;
        speedR=maxSpeed/ratio;
        analogWrite(6, speedL); //left motor speed
        analogWrite(5, speedR); //right motor speed
        digitalWrite(7, LOW); //left motor directiom
        digitalWrite(4, LOW);
        distRight = (-PololuWheelEncoders::getCountsM1());
        distLeft = (PololuWheelEncoders::getCountsM2());
        Setpoint2 = 0;
        while((distLeft<outerRad  ||  distRight<innerRad)){
          Input2 = distRight*ratio-distLeft;
          turningPID.Compute();
          speedR=255+Output2;
          analogWrite(6, 255); //left motor speed
          analogWrite(5, abs(speedR)); //right motor speed
          digitalWrite(7, LOW); //left motor directiom
          if(speedR<0){ digitalWrite(4, HIGH); }
          else        { digitalWrite(4, LOW);  }
          distRight = (-PololuWheelEncoders::getCountsM1());
          distLeft = (PololuWheelEncoders::getCountsM2());
        }
      case -2: //Pull a u-turn. Similiar to above two but radiuses are doubled.
        analogWrite(6, 200); //left motor speed
        analogWrite(5, 220); //right motor speed
        digitalWrite(7, LOW); //left motor directiom
        digitalWrite(4, LOW); //right motor direction
        delay(300);
        rad+=2;
        innerRad = (rad+Setpoint)*pi/4*encoderUnitValue;
        outerRad = (rad+Setpoint+20)*pi/4*encoderUnitValue;
        ratio = outerRad/innerRad;
        innerRad=innerRad;
        outerRad=outerRad;
        speedL=maxSpeed;
        speedR=maxSpeed/ratio;
        analogWrite(6, speedL); //left motor speed
        analogWrite(5, speedR); //right motor speed
        digitalWrite(7, LOW); //left motor directiom
        digitalWrite(4, LOW);
        distRight = (-PololuWheelEncoders::getCountsM1());
        distLeft = (PololuWheelEncoders::getCountsM2());
        Setpoint2 = 0;
        while((distLeft<outerRad  ||  distRight<innerRad)){
          Input2 = distRight*ratio-distLeft;
          turningPID.Compute();
          speedR=255+Output2;
          analogWrite(6, 255); //left motor speed
          analogWrite(5, abs(speedR)); //right motor speed
          digitalWrite(7, LOW); //left motor directiom
          if(speedR<0){ digitalWrite(4, HIGH); }
          else        { digitalWrite(4, LOW);  }
          distRight = (-PololuWheelEncoders::getCountsM1());
          distLeft = (PololuWheelEncoders::getCountsM2());
        }
    }
    motorFullStop();	//Used primarily for debugging/outputting but found helps eliminate error.
    delay(2000);		//Delay added for visual recognition that it has completed its turn.
    PololuWheelEncoders::getCountsAndResetM2();	//reset encoder counts for next execution, or non-turning use.
    PololuWheelEncoders::getCountsAndResetM1();
}
*/
```

Correction function that allows the robot to adjust to errors that occur during the run. Will be further updated to call distance values and smartly correct further.
```
bool entered = false; //Used to indicate whether the robot has entered the bottom of the maze and has started following the right wall. Deprecated as it was not fully developed yet.
void correct() {	//Correction function that allowed the robot to correct itself according to which wall it was following in the optimal way according to test observations.
  int randNumber = random(5, 12);	//RNG used to ensure that correction doesn't fall into a back/forth loop.
  analogWrite(6, speed); //left motor speed
  analogWrite(5, speed*1.15); //right motor speed
  digitalWrite(7, HIGH); //left motor direction
  digitalWrite(4, HIGH); //right motor direction
  delay(500);	//Runs the motor backwards for a bit.
  if (entered==true){	//Following left wall. Correct by turning right.
    analogWrite(6, speed); //left motor speed
    analogWrite(5, speed*randNumber/20); //right motor speed
  } else {	//Following right wall. Correct by turning left.
    analogWrite(6, speed*randNumber/20); //direction reversed once following other side
    analogWrite(5, speed); 
  }
  digitalWrite(7, LOW); //left motor direction
  digitalWrite(4, LOW); //right motor direction
  delay(50);	//Step forward then collision detection detects takeover to detect before front collision.
}
```

Declare modular turn functions that can be executed by the robot when it senses the need for a right or left turn. Hard coded turns as encoder turning was not gonna be successful unless encoder could also control the speed at which to turn. As PID of turning with encoder function is too rough. This was very beneficial as it cut out crucial time to get to the miner and because the PID was controlling before miner identification it helps identify the miner more consistently.
```
void turnRight(){
  speedL=220;
  speedR=speed*78/200; //Slower right turn speed leads to a smooth sweeping turn rather than a tank turn.
  analogWrite(6, speedL); //left motor speed
  analogWrite(5, speedR); //right motor speed
  digitalWrite(7, LOW); //left motor direction
  digitalWrite(4, LOW); //right motor direction
  delay(250);
//  Serial.print("\n");	//Following code was used to measure distance to the left through the halfway point of the turn to determine if there was nothing there.
//  Serial.println(distance(0));
//  if(distance(0)>28){	//If so then changed entered to true, and reverse above movement.
//    entered=true;
//    analogWrite(6, speedL); //left motor speed
//    analogWrite(5, speedR); //right motor speed
//    digitalWrite(7, HIGH); //left motor directiom
//    digitalWrite(4, HIGH);
//    delay(250);
//    motorFullStop();	//After reversing above, stop and turn left and move forward a tad to begin following left wall at bottom of maze.
//    turnLeft();
//    analogWrite(6, speed); //left motor speed
//    analogWrite(5, speed); //right motor speed
//    digitalWrite(7, LOW); //left motor directiom
//    digitalWrite(4, LOW);
//    delay(180);
//  } else {	//or else continue turning as you were and move slightly forward.
    analogWrite(6, speedL); //left motor speed
    analogWrite(5, speedR); //right motor speed
    digitalWrite(7, LOW); //left motor directiom
    digitalWrite(4, LOW);
    delay(300);
    analogWrite(6, speed); //left motor speed
    analogWrite(5, speed); //right motor speed
    digitalWrite(7, LOW); //left motor directiom
    digitalWrite(4, LOW);
    delay(180);
//	}
}

void turnLeft(){
  speedR=255;
  speedL=50;
  Serial.print("\nLeft turn being executed\n");
  analogWrite(6, speedL); //left motor speed
  analogWrite(5, speedR); //right motor speed
  digitalWrite(7, LOW); //left motor directiom
  digitalWrite(4, LOW);
  delay(150);
//  if(distance(2)>15){	// Similiar to above, but we're making sure that the right is free after executing 25% of the turn.
//    entered=true;	//If true start following the right wall.
//    Serial.println("\nChanged entered value!\n");
//  }
  delay(450); //Either way continue turning left and then start following either wall.
}
```

Main PID declaration for ensuring straight line stability. PID features a dual-stage allowing the other stage to provide a quicker recovery and being able to run lower values for the main PID allows for a more stable straight-line performance. PID was tuned using excel and outputs from serial.
```
double Kps = 10, Kis = 8, Kds = 2;	//Normal PID parameters
double Kpt = 15, Kit = 13, Kdt = 3;	//Aggressive PID parameters
double Setpoint, Threshold, Input, Output;	 //declare necessary variables for PID
double Kp, Ki, Kd;
PID straightPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);	//Setup PID to be called later
```

# Main Setup function.
```
void setup() {
  PololuWheelEncoders::init(10, 11, 12, 13);	//Initialize encoder ports
  delay(3000);
  Serial.begin(9600);
  pinMode(6, OUTPUT);	//Initialized pins for motor control.
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(trig, INPUT);
  Setpoint=7.5;	//Setpoint for straightline PID (aka distance from left/right wall)
  //Setpoint=(distance(0)+distance(2))/2;	//Deprecated. Allows for guesswork to be thrown out window as it measures both walls. Havent been able to reliabaly scale PID to this value.
  straightPID.SetMode(AUTOMATIC);	//Start straight and deprecated turning PID.
  //turningPID.SetMode(AUTOMATIC);
}
```

# Main loop function.
```
int x = 0;	//Initialize x variable (used to measure unsuccessful turn attempts and used to execute correct function later)
void loop() {
      if (abs(x)>2){	//If three unsuccessful turn attempts are made call correction function.
        correct();	//eliminate errors by introducing random correction.
        x=0;	//Reset unsuccessful turn count.
      }
      distanceL=distance(0);	//Distance from the wall on the left.
      distanceR=distance(2);	//Distance from the wall on the right.
      distanceC=distance(1);	//Distance from the wall on the centre.
      distanceM=distance(3);	//Distance from the wall on the miner (lower centre position).
      Serial.print(distanceL);	//Output values to understand how code thinks.
      Serial.print(" <- Left distance Right -> ");
      Serial.println(distanceR);
      Serial.print(distanceC);
      Serial.print(" <- Centre distance Bottom -> ");
      Serial.println(distanceM);
//      int distRight = (-PololuWheelEncoders::getCountsM1());	//deprecated code that measures how much encoder has turned since last turn or beginning of course if no turn has occured.
//      int distLeft = (PololuWheelEncoders::getCountsM2());
//      Serial.print((distRight+distLeft)/2);
//      Serial.print("\t");
//      Serial.println(distanceR/Setpoint);
      if ((lastDistM+1)>distanceM && distanceM<8 && lastDistM<8 && distanceC>(distanceM+3) && lastDistC>(distanceM+3)){ 
        motorFullStop();	//If miner distance is less than 8 and the centre distance is greater than 3 compared to miner distance then execute the following code. 
        delay(3000);		//Stops long enough to register that the robot has found the miner
      }
      if (distanceC>18){	//If nothing in front of the robot
        speedL=speed;	//Set both motors to max speed allowed. Declared earlier.
        speedR=speed;
        if((distanceR>=6 && distanceR<9)){	//If within range 3 of setpoint value use reg PID
          if (entered==false){	//If following right wall input is ...
            Input=distanceR;
          } else {
            Input=15-distanceL;	//If following left wall input is ...
          }
          straightPID.SetTunings(Kps, Kis, Kds);	//Set reg PID settings and output setting to make sense.
          straightPID.SetOutputLimits(-2*speed,2*speed);
          straightPID.Compute();
          //Serial.print("\t");		//Used to test performance of PID in excel
          //Serial.println("Reg");
          if(Output<0){	//Adjust speeds accordings to output
            speedR = speed+Output;
          } else {
            speedL = speed-Output;
          }
          x=0;	reset mistake counter.
        } else if (distanceR<6 || (distanceR<=16 && distanceR>=9)){ //If within cell range but not within sweet spot then use more aggressive PID controllers.
          if (entered==false){ //If following right wall input is ...
            Input=distanceR;
          } else {
            Input=15-distanceL; //If following left wall input is ...
          }
          straightPID.SetTunings(Kps, Kis, Kds);
          straightPID.SetOutputLimits(-2*speed,2*speed);
          straightPID.Compute();
          if(Output<0){
            speedR = speed+Output;
          } else {
            speedL = speed-Output;
          }
          //Serial.print("\t");
          //Serial.println("Aggresive");
          x=0;
        } 
        else if (distanceR>=15 && entered==false){	//If right is open and we're following the right wall then turn right
          motorFullStop();
          turnRight();
          x=x+1;
        }
        else if (distanceL>=15 && entered==true){	//If left is open and we're following left turn then turn left.
          motorFullStop();
          turnLeft();
          x=x-1;
        }
        boolean directionL = LOW; //set to forward
        boolean directionR = LOW;
        if(speedL<0){ directionL = HIGH; }
        if(speedR<0){ directionR = HIGH; }
        analogWrite(6, abs(speedL)); //left motor speed
        analogWrite(5, abs(speedR)*1.15); //right motor speed
        digitalWrite(7, directionL); //left motor directiom
        digitalWrite(4, directionR);
      }
      else if (distanceC>8 && distanceC<=18) {	//If robot is within 8-18cm from the wall.
        speedL=speed;
        speedR=speed;
        if(distanceL<12 && distanceR<12){	//If no wall on either side then turn 180 degrees.
          if(lastDistL>lastDistR){
            speedL=speedL*(-1);
          } else {
            speedR=speedR*(-1);
          }
          x=0;
        } else if (distanceR>=12 && distanceL>=15){ //If both left and right are open then turn left as it saves more time. Only happend in one scenario at bottom of the map.
          motorFullStop();
          turnLeft();
          x=x-1;
        } else if (distanceR>=12){	//Turn right if right distance rises
          motorFullStop();
          turnRight();
          x=x+1;
        } else if (distanceL>=12){	//Turn left if left distance is high
          motorFullStop();
          turnLeft();
          x=x-1;
        }
        boolean directionL = LOW; //set to forward
        boolean directionR = LOW;
        if(speedL<0){ directionL = HIGH; }
        if(speedR<0){ directionR = HIGH; }
        analogWrite(6, abs(speedL)); //left motor speed
        analogWrite(5, abs(speedR)*1.15); //right motor speed
        digitalWrite(7, directionL); //left motor directiom
        digitalWrite(4, directionR); //right motor direction
      } else {	// If nothing can be done than correct until something can be done.
        correct();
        x=x+1;
      }
      lastDistL=distanceL;	//Store most recent values for IR sensors and restart loop.
      lastDistR=distanceR;
      lastDistC=distanceC;
      lastDistM=distanceM;
}
```