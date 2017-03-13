# AMITRobot
AMITRobot stands for Autonomous Mining Intervention Technology Robot.

# Initial Library Setup
Ensure that all libraries are placed within respective folders to ensure proper running of code. Note that the Pololu wheel encoder library goes under Arduino Folder>Hardware instead of Folder>Libraries.

Phase B: Servo library included for ease of use of servo for claw mechanism
```
#include <PololuWheelEncoders.h>
#include <PID_v1.h>
#include <Servo.h>
```

# Variable Decleration
Declare Servo variable, initial angle, and function to control the motion of the servo. 
```
Servo claw;  
int i = 180;

void closeClaws(){
    for (i = 180; i >=110 ; i -=1) { //Close claws from 180 degrees to 110 degrees.
      claw.write(i);
      //Serial.println(i);          
      delay(50);     		     //Delay until claws reach the desired degrees.                  
    }
}
```

# Navigation Variables
Moving away from the wall following design to facilitate proper navigation to succesfully get to miner. Directions are given to robot interms of turning and distances to travel before searching for miner or ending of maze.
```
int minerDist = 2550;	// Miner distance and below an array that contains navigation instructions -1 = right turn, 1 = left turn, val = distance to travel
int routeDirections[] = {-1,-1,1,1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1, minerDist, 1,1,-1,-1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,-1,1,1,minerDist*2};
//int routeDirections[] = {1, 1, 1, -1}; //Can be used to test different routes or small areas. 
int x=0;		// Current direction following. Allows PID to be tuned to suit specific wall and its characteristics.

bool turning = false;	// Is robot currently turning. Used for loop control
bool gotMiner = false;	// Does the robot have the miner? Used for claw control.
float angleToTurn = 0;	// Used to tell robot how much to turn. Adjusts 90 degrees turns to accomodate for entry angle.
```

# Speed Control Variables 
Will be used to control outputs to individual motors from PID, turning or stopping functions.
```
int speedLBase = 125;		//Used as baseline to control rpm of Left/Right wheel using Proportional controller.
int speedRBase = speedLBase;
int speedL = speedLBase;	//Individual speed sent to left/right motor ports.
int speedR = speedRBase;
float speedLeft, speedRight;	//Speed needed from the motors in encoder units / second.
float Vr, Vl;			//Current speed of the motors in encoder units / second.
long int deltaTime;		//Delta time used in above speed calculation.
int deltaRDist, deltaLDist	//Delta distance used in above speed calculation.
int totalDistR, totalDistL;	//Total distance covered by each encoder. Used for comparisons in turning and averages to accurately measured distance travelled.
long int lastTime;		//Used to calculate delta time.
```

# Modular Function Decleration
Declare distance variables for each IR sensor and distance calculation function that is used throughout the code for calculating distance from wall. This function was calibrated and developed seperately using pySerial output with polynomial regression to develop a more accurate function than online versions. 

Phase B: Note the addition of a new IR sensor and the switch over to using an array of values instead of individual variables. The plan was to use the difference between front and back to create a yaw angle but there was too much error for the value and further development will be required for phase C.
```
float distances[5];
float lastDistances[5];
float distance(int analogInput) { //returns IR sensor reading when provided port number.
  double volts = (analogRead(analogInput)) * 0.0048828125;
  double dist = -(2.01423036 * pow(volts, 5)) + (19.44766216 * pow(volts, 4)) - (73.53682794 * pow(volts, 3)) + (138.47666662 * pow(volts, 2)) - (136.60286212 * pow(volts, 1)) + 66.31890266;
  return dist;
}
```

Declare stop function that writes to both motor power outputs M1,M2 a value of zero indicating a hard stop for both wheels. Used for crash prevention and stopping for miner.
```
void motorFullStop() {	//No parameters needed simply call function
  analogWrite(6, 0);	//Right motor power = 0
  analogWrite(5, 0);	//Left motor power = 0
}
```

Phase B: Unlike our previous design where the correction function allowed the robot to recover itself and continue wall following. The new design with the ability to move away from wall following and provide turn by turn decision making requires more chronological control. Therefore correction code is limited here as a hail mary attempt to save a turn. However this didn't work particularly effectively. 

Phase C: Definently needs to be implemented to prevent errors and potentially recover from potential mistakes. 
```
void correct() {
    motorFullStop();
    digitalWrite(7, HIGH);
    digitalWrite(4, HIGH);
    analogWrite(6, 82);
    analogWrite(5, 75);
    delay(600);			//Back up for 0.6 sec if close to the wall in front. 
    digitalWrite(7, LOW);	//Reset the travel direction as forward and continue turning.
    digitalWrite(4, LOW);
}
```

Phase B: Function that allows us to provide a desired speed in encoder units/sec. Basically an RPM and the PID will work towards maintaning that. Moved away from PID library and created own proportional controller which worked best over integral and derivative controller due to higher hz.
```
void calcMotorSpeed(float desiredSpeedL, float desiredSpeedR){		// Desired speeds are passed through by outer PID controlling individual speeds
//  SetpointL = desiredSpeedL;
//  SetpointR = desiredSpeedR;
  deltaRDist = -PololuWheelEncoders::getCountsAndResetM2();		// Difference in distance travelled by each encoder.
  deltaLDist = -PololuWheelEncoders::getCountsAndResetM1();		// Resets allow us to calculate difference without storing value
  totalDistR=totalDistR+deltaRDist;					// Total distance stored as value for later calculations.
  totalDistL=totalDistL+deltaLDist;
  deltaTime = micros()-lastTime;					// Difference in time calculated
  lastTime = micros();							// New current time is stored for later difference in time calculation
  Vr = (float)deltaRDist / (float)deltaTime * 1000000;			// Current speed of motor for left/right encoder
  Vl = (float)deltaLDist / (float)deltaTime * 1000000;
  speedL=max(0,min(round(speedLBase+KpL*(desiredSpeedL-Vl)), 255));	// Current speed of left and right motor (value from 0,255)
  speedR=max(0,min(round(speedRBase+KpL*(desiredSpeedR-Vr)), 255));	// Note adjustment of speedRbase instead of direct assignment
  /*
   //put serial prints here. Too many had to delete. Used for testing purposes.
  */
  speedLBase=speedL;	//Crucial part allows PID to operate well on different speeds. A baseline speed is given and then basically "reassessed" above
  speedRBase=speedR;	//Allows it to gradually change speeds along with the required speed. Changing outer Kp just makes each more reactive.
}
```

Main PID declaration for ensuring straight line stability. PID features a dual-stage allowing the other stage to provide a quicker recovery and being able to run lower values for the main PID allows for a more stable straight-line performance. PID was tuned using excel and outputs from serial.
```
double Setpoint, Threshold, Input, Output;
double KpL; 		//Note that RPM of Left/Right motor is simply controlled by a proportional controller at approximately 48 hz.
double KpR;
double Kp, Ki, Kd;	//PID to control required RPM of either wheel to ensure corrections and turns are followed accurately.
PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
```

# Main Setup Function
Setup function declares all pins and initial tuning settings for PIDs.
```
void setup() {
  PololuWheelEncoders::init(8,9,10,11);	//Initialize encoder ports
  delay(2000);
  Serial.begin(9600);
  pinMode(6, OUTPUT);	//Initialized pins for motor control.
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);
  claw.attach(13); 	//Create port to write for claw movement.
  digitalWrite(7, LOW);	//Set original direction of motors.
  digitalWrite(4, LOW);
  PID.SetMode(AUTOMATIC);
  KpR = 0.005;		//Kp values for left/right motor encoder PID
  KpL = 0.005;
  Kp=80;		//Setup wall following PID value. Will be changed throughout code.
  Ki=0;
  Kd=30;
  PID.SetTunings(Kp, Ki, Kd);
  PID.SetOutputLimits(-1000, 1000);
  claw.write(180);  	//Keep claw open.
  Setpoint = 2.3;	//Setpoint of first wall follow. Value will be changed accordingly after.
  lastTime = micros();	//Setup lasttime for accurate time measurements after setup
}
```

# Main Loop Function
```
void loop() {
  distances[0] = distance(1)-3;		//Distance of right wall
  distances[1] = distance(2)-3;		//Distance of front wall
  distances[2] = distance(3)-3;		//Distance of left wall
  distances[3] = distance(4);		//Distance of miner 
  //distances[4] = distance(5)-2.13;	//Used to measure current angle of robot. Eliminated due to large error in angle measurement.
  /*					//Used to measure average distance of robot from wall for fine-tuning setpoints.
  Serial.print(distances[0]);
  x++;
  Serial.print("/");
  Serial.println((total+=distances[0]));
  */

  if(turning==false){
    //If robot is not currently turning. 

    if(routeDirections[x]<2 && distances[1+routeDirections[x]]>12){
	//If current navigation instruction is not a distance and next navigation instruction path is clear ...	

      Serial.print("Detected turn, turning: ");
      Serial.println(routeDirections[x]);
      motorFullStop();				//Stop
      turning=true;				//Set turning to true to operate loop control
      angleToTurn=abs(1900*routeDirections[x]-(totalDistR-totalDistL));	//begin turning 90 degrees plus/minus current angle according to encoders
      totalDistR = 0;				//Reset current distance to ensure 90 degrees turn is executed perfectly.
      totalDistL = 0;
      return;					//Exit rest of loop execution
    }
    else{
	//Current navigation is still needed and next step hasn't been reached so ...

      Serial.println("Currently PIDing");
      Input=(distances[0]);	//Scan right wall for distance
      if(distances[0]>Setpoint && distances[0]<6){	//If robot too far left of setpoint PID as such ...
        Kp=120;
        PID.Compute();
        speedLeft = max(0,min(1000, 1000-Output));	//Assign value for desired encoder speed left/right
        speedRight = max(0,min(1000, 1000+Output));
      } else if (distances[0]<Setpoint && distances[0]>0){  //If robot too close to wall compared to setpoint ...
        Kp=600;
        PID.Compute();
        speedLeft = max(0,min(1000, 1000-Output));	//Assign value for desired encoder speed left/right
        speedRight = max(0,min(1000, 1000+Output));
      } else {						//Robot is in a really bad position right now ...
        Kp=550;
        PID.Compute();
        speedLeft = max(0,min(800, 800-Output));	//Assign value for desired encoder speed left/right
        speedRight = max(0,min(800, 800+Output));
      }
      calcMotorSpeed(speedLeft, speedRight);		//Send desired speeds to motor rotation controller
      /*						//Outputted to confirm PID functionality
      Serial.print(speedLeft);
      Serial.print("\t");
      Serial.print(Vl);
      Serial.print("\t");
      Serial.print(speedRight);
      Serial.print("\t");
      Serial.println(Vr);
      */
      analogWrite(6, speedL);				//Write motor speed values returned by encoder control
      analogWrite(5, speedR);
      if(routeDirections[x]>2){
	//If current navigation instruction is a distance

        if(((lastDistances[3] + 1) > distances[3] && distances[3] < 4 && lastDistances[3] < 5 && distances[1] > 3 && lastDistances[1] > 3)&& gotMiner == false){
	  //bottom sensor detects miner and stops
	  //front sensor detects something that is closer than 1cm and now must correct.
          Serial.println("Detected Miner");
          motorFullStop();
          delay(4000); 
          gotMiner = true;	// Notify loop control we have the miner
          closeClaws();   	// Close the claws to secure miner
	  x++			// Move onto next navigation instruction
        } else if (gotMiner==true && ((totalDistR-totalDistL)/2>routeDirections[x])){
	  motorFullStop();	// If miner already retrieved stop 
          delay(4000);
	  x++			// Move onto next navigation instruction
        } else if ((totalDistR+totalDistL)/2>routeDirections[x]){
          motorFullStop();
          delay(4000);
	  x++			// Move onto next navigation instruction
        }
      }
    }
  }
  else {
	// Robot is currently turning

    if(distances[1]<2){	//If wall is detected infront back up a little and correct mistake
      Serial.print(distances[1]);
      Serial.println("\tDetected Wall in front!");
      correct();	//Call correction function here
    }
    if(abs(totalDistR-totalDistL)<angleToTurn){
	//if difference between both wheel rotations is not enough to have completed the required angle to turn then ...

      Serial.print("Currently turning: ");
      Serial.print(routeDirections[x]);
      Serial.print("\tAmount Turned: ");
      Serial.println(abs(totalDistR-totalDistL));
      calcMotorSpeed(520-280*routeDirections[x],520+280*routeDirections[x]);	// Turn left or right wheel accordingly to make turn
      analogWrite(6, speedL);	//Write output left/right speeds from encoder control function.
      analogWrite(5, speedR);
    }
    else {
	//if left and right wheels have turned enough to create the 90 degree turn then...

      analogWrite(6, 70);	//Move forward slightly and then PID if necessary to the width of the wall or post
      analogWrite(5, 62);
      delay(300+25*routeDirections[x]);
      motorFullStop();		//Signify end of turn so user can evaluate turn quality and record and feedback
      delay(1000);
      turning=false;		//Set turning back to false, so next navigation instruction can be followed.
      totalDistR=0;		//Distances set back to zero in case next navigation is a distance.
      totalDistL=0;
      Serial.println("Finished turning");
      x+=1;			//Move onto next navigation instruction and assign setpoint for next navigation below
      switch (x) {		
	//Each wall has its unique flavour of distance that allows us to strategically place our robot in its centre
	//The following values were tested and provided necessary precision to accurately turn. 

        case 2: Setpoint = 2.4;
//        case 3:
        case 4: Setpoint = 2;
//        case 5: 
        case 6: Setpoint = 2.9;
//        case 7:
        case 8: Setpoint = 2.3;
        case 9: Setpoint = 2;
        case 10: Setpoint = 3.2;
        case 11: Setpoint = 2;
        case 12: Setpoint = 2.9;
//        case 13: 
        case 14: Setpoint = 2;
        case 15: Setpoint = 3;
//        case 16:
        case 17: Setpoint = 2.2;
//        case 18:
//        case 19:
//        case 20:
        case 21: Setpoint = 3;
        case 22: Setpoint = 3.5;
        case 23: Setpoint = 4;
        case 24: Setpoint = 4.5;
        case 25: Setpoint = 5;
        case 26: Setpoint = 5;
        case 27: Setpoint = 5;
        default: Setpoint = 2.3;
      }
    }
  }

  lastDistances[0] = distances[0];	//Assign last values read by IR sensors for loop control to call later
  lastDistances[1] = distances[1];
  lastDistances[2] = distances[2];
  lastDistances[3] = distances[3];
  //lastDistances[4] = distances[4];
}
```