#include <Wire.h>
#include <Zumo32U4.h>


Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;


#define NUM_SENSORS 5                    //defines how many sensors is active.
uint16_t lineSensorValues[NUM_SENSORS];  //Int-array with the line sensor readings.

float speed = 150.0;           //Standard speed
float sharpTurnInside = -0.2;  //Inside wheel speed factor on sharp turns.
float sharpTurnOutside = 0.5;  //Outside wheel speed factor on sharp turns.
float softTurnInside = 0.5;    //Inside wheel speed factor on soft turns.
float softTurnOutside = 0.75;  //Outside wheel speed factor on soft turns.
float turnInside = 0.3;        //Inside wheel speed factor on normal turns.
float turnOutside = 0.75;      //Outside wheel speed factor on normal turns.
bool driveMode = 0;            //Drivemode. If drivemode is 1, a while-loop with the drive algorithm will run. If 0, the program will exit the while-loop.
bool lastTurn = 0;             //This will keed track of the last sharp turn, incase the line should enter a blindspot between the sensors.

uint16_t lineSensorMax[5] = { 0, 0, 0, 0, 0 };                 //Int-array with the max value each sensor has read, during the calibration funktion. The values start at zero, to be up-adjusted in the calibration function.
uint16_t lineSensorMin[5] = { 2000, 2000, 2000, 2000, 2000 };  //Int-array with the min value each sensor has read, during the calibration funktion. The values start at max, to be down-adjusted in the calibration function.
uint16_t threshold[5] = { 800, 600, 450, 600, 800 };           //Int-array with the thesholds for when a sensor's values qualifies as on the line or not.


void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();  //Initialize five line sensors.
  Serial.begin(9600);             //Begin serial communication.
}

void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  //Reads line sensors.
  //printReadingsToSerial(); //Prints readings to serial.
}

void printReadingsToSerial() {  //The funktion that prints readings to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          lineSensorValues[0],  //left sensor
          lineSensorValues[1],  //middle left
          lineSensorValues[2],  //middle
          lineSensorValues[3],  //middle right
          lineSensorValues[4]   //right sensor
  );
  Serial.print(buffer);
}
void printMaxToSerial() {  //The funktion that prints the max values to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          lineSensorMax[0],  //left sensor
          lineSensorMax[1],  //middle left
          lineSensorMax[2],  //middle
          lineSensorMax[3],  //middle right
          lineSensorMax[4]   //right sensor
  );
  Serial.print(buffer);
}
void printMinToSerial() {  //The funktion that prints the min values to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          lineSensorMin[0],  //left sensor
          lineSensorMin[1],  //middle left
          lineSensorMin[2],  //middle
          lineSensorMin[3],  //middle right
          lineSensorMin[4]   //right sensor
  );
  Serial.print(buffer);
}
void printThresholdToSerial() {  //The funktion that prints the theshold values to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          threshold[0],  //left sensor
          threshold[1],  //middle left
          threshold[2],  //middle
          threshold[3],  //middle right
          threshold[4]   //right sensor
  );
  Serial.print(buffer);
}


//Movement funktions. Self explanetory. -----------------------------------------------------------------------
void forward() {
  //display.print("WUUUU!!!");
  display.print("fwd");
  motors.setSpeeds(speed, speed);
}
void sharpLeft() {
  display.print("<----!!!");
  motors.setSpeeds(speed * sharpTurnInside, speed * sharpTurnOutside);
}
void sharpRight() {
  display.print("!!!---->");
  motors.setSpeeds(speed * sharpTurnOutside, speed * sharpTurnInside);
}
void turnLeft() {
  display.print("<---");
  motors.setSpeeds(speed * turnInside, speed * turnOutside);
}
void turnRight() {
  display.print("--->");
  motors.setSpeeds(speed * turnOutside, speed * turnInside);
}
void softLeft() {
  display.print("<--");
  motors.setSpeeds(speed * softTurnInside, speed * softTurnOutside);
}
void softRight() {
  display.print("-->");
  motors.setSpeeds(speed * softTurnOutside, speed * softTurnInside);
}
void spin() {
  display.print("spin");
  motors.setSpeeds(speed * 0.5, -speed * 0.5);
}



void calibrate() {
  display.clear();
  display.print("delay");
  delay(1000);
  display.clear();
  int i = 0;
  int j = 0;

  spin();                       //Sets the robot in a spin, so the sensors will pass over the line and the floor.
  for (i = 0; i < 5000; i++) {  //This loop runs 5000 times. Each sensor is read once per loop, and the min/max values are updated if needed.
    readLineSensors();
    for (j = 0; j < 5; j++) {                        //For loop that iterates through each line sensor.
      if (lineSensorValues[j] > lineSensorMax[j]) {  //Checks if the newly read line sensor value is higher than the previous max.
        lineSensorMax[j] = lineSensorValues[j];      //Sets max line sensor value to the newly read line sensor value.
      }
      if (lineSensorValues[j] < lineSensorMin[j]) {  //Checks if the newly read line sensor value is lower than the previous min.
        lineSensorMin[j] = lineSensorValues[j];      //Sets min line sensor value to the newly read line sensor value.
      }
    }
    Serial.print("Max:     ");
    printMaxToSerial();  //Prints provisional max values
    Serial.print("Readings:");
    printReadingsToSerial();  //Prints readings
    Serial.print("Min:     ");
    printMinToSerial();  //Prints provisional min values
  }
  motors.setSpeeds(0, 0);  //Stops the spin. Max and min values are found at this point.
  j = 0;
  for (j = 0; j < 5; j++) {                                                       //Iterates through each line sensor
    threshold[j] = (lineSensorMax[j] - lineSensorMin[j]) / 2 + lineSensorMin[j];  //Calculates the thesholds based on max and min.
  }
  Serial.print("Thres:  ");
  printThresholdToSerial();  //Prints theshold.
}

void drive() {
  while (driveMode) {
    readLineSensors();
    display.clear();
    if (lineSensorValues[2] >= threshold[2] && lineSensorValues[1] >= threshold[1] && lineSensorValues[0] >= threshold[0]) {         //Checks if sensor 0, 1 and 2 are on a line, to determine if sharp turn should be executed.
      sharpLeft();                                                                                                                   //Sharp turn left
    } else if (lineSensorValues[2] >= threshold[2] && lineSensorValues[3] >= threshold[3] && lineSensorValues[4] >= threshold[4]) {  //Checks if sensor 0, 1 and 2 are on a line, to determine the uther sharp turn.
      sharpRight();                                                                                                                  //The other sharp turn.
    } else if (lineSensorValues[2] >= threshold[2] && lineSensorValues[1] >= threshold[1]) {                                         //Checks if line sensor 1 and 2 is on a line, to determine if a soft turn should be performed.
      softLeft();                                                                                                                    //Soft turn left
    } else if (lineSensorValues[2] >= threshold[2] && lineSensorValues[3] >= threshold[3]) {                                         //Soft turn to the other side. Sensor 2 and 3.
      softRight();                                                                                                                   //Soft turn
    } else if (lineSensorValues[1] >= threshold[1]) {                                                                                //Checks sensor 1 to perform normal turn left.
      turnLeft();                                                                                                                    //Normal turn left
    } else if (lineSensorValues[3] >= threshold[3]) {                                                                                //Checks sensor 3 to perform normal turn right.
      turnRight();                                                                                                                   //Turn funktion right
    } else if (lineSensorValues[2] >= threshold[2]) {                                                                                //If only the middle sensor is on the line, drive forward.
      forward();                                                                                                                     //self
    } else if (lineSensorValues[0] >= threshold[0]) {                                                                                //If ONLY the far left sensor (sensor 0) is on the line, make sharp turn left and remember that the last turn was left.
      sharpLeft();                                                                                                                   //Sharp left
      lastTurn = 1;                                                                                                                  //If lastTurn is 1, the last turn was sharp left. If 0, the last turn was right. This is used to keep turning, should the line enter the blindspot between sensor 0 to 1 and 3 to 4.
    } else if (lineSensorValues[4] >= threshold[4]) {                                                                                //If ONLY the far left sensor (sensor 4)
      sharpRight();                                                                                                                  //Sharp right
      lastTurn = 0;                                                                                                                  //same as above
    } else {                                                                                                                         //This will be performed of no sensor is on the line (The blindspot).
      display.print("blind");                                                                                                        //Prints state
      if (lastTurn == 1) {                                                                                                           //Determines what the last sharp turn was, and keeps doing that.
        sharpLeft();
      } else {
        sharpRight();
      }
    }
    if (buttonC.getSingleDebouncedRelease()) {  //This button will set drivemode to 0, and the loop will therefore not continue.
      driveMode = 0;                            //yes
      motors.setSpeeds(0, 0);                   //To stop whatever movement is going on
      buttonC.waitForRelease();
    }
  }
}


void loop() {

  display.clear();

  if (buttonA.getSingleDebouncedRelease()) {
    buttonA.waitForRelease();
    calibrate();
  }

  if (buttonB.getSingleDebouncedRelease()) {
    driveMode = 1;
    buttonB.waitForRelease();
    drive();
  }
  /*
  if (buttonA.getSingleDebouncedRelease()) {
    if (lineSensorValues[2] >= threshold[2] && lineSensorValues[1] >= threshold[1] && lineSensorValues[0] >= threshold[0]) {
      sharpLeft();
    } else if (lineSensorValues[2] >= threshold[2] && lineSensorValues[3] >= threshold[3] && lineSensorValues[4] >= threshold[4]) {
      sharpRight();
    } else if (lineSensorValues[2] >= threshold[2] && lineSensorValues[1] >= threshold[1]) {
      softLeft();
    } else if (lineSensorValues[2] >= threshold[2] && lineSensorValues[3] >= threshold[3]) {
      softRight();
    } else if (lineSensorValues[1] >= threshold[1]) {
      turnLeft();
    } else if (lineSensorValues[3] >= threshold[3]) {
      turnRight();
    } else if (lineSensorValues[2] >= threshold[2]) {
      forward();
    } else if (lineSensorValues[0] >= threshold[0]) {
      sharpLeft();
      lastTurn = 1;
    } else if (lineSensorValues[4] >= threshold[4]) {
      sharpRight();
      lastTurn = 0;
    } else {
      display.print("blind");
      if (lastTurn == 1) {
        sharpLeft();
      } else {
        sharpRight();
      }
    }
  }
  */
}
