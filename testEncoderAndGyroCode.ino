#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;

unsigned long startTime;  //Used to timesptamp the gyro tests.
int16_t lastAngle = 0;   //Used to check if the angle has changed in the gyro tests (printGyroChange).

float speed = 150.0;  //Standard speed. Float so that it can be devided.

//Encoder calibration variables. Encoders is calibrated by utilizing the existing drive functions.
uint16_t countsToMeasureDistanceOf = 5000;  //Counts to measure distance of.
uint16_t measuredDistance = 64;             //The measured distance for calibrating driveDistance accuracy.

//Gyro variables. Gyro code is at the bottom of the document.
int16_t gyroOffset;
uint32_t turnAngle = 0;
int16_t turnRate;
uint16_t gyroLastUpdate = 0;
uint16_t rounds = 20000;  //Amount of times it runs the calibration cycle.

void setup() {
  Serial.begin(9600);  //Begin serial communication.
}
void displayAngle() {  //Displays angle on display.
  display.gotoXY(0, 0);
  display.print(getTurnAngleInDegrees());
  display.print(F("   "));
}
void displayCounts() {  //Displays counts on display.
  display.gotoXY(0, 1);
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  if (encoders.getCountsRight() > 999 || encoders.getCountsLeft() > 999 || encoders.getCountsLeft() < -99 || encoders.getCountsRight() < -99) {
    countsL = encoders.getCountsLeft() / 10;
    countsR = encoders.getCountsRight() / 10;
  }
  String waka = (String)countsL + "/" + (String)countsR;
  display.print(waka);
  display.print(F("   "));
}
void driveStraightEncoder(int driveDistance) {  //Drive straight according to encoders. The driveDistance input is how long it will drive in counts.
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
  motors.setSpeeds(speed, speed);
  while (encoders.getCountsLeft() < driveDistance && encoders.getCountsRight() < driveDistance) {
    displayAngle();
    displayCounts();
    if (encoders.getCountsLeft() > encoders.getCountsRight() + 10) {  //If gyroscope angle is more than one degree off, the bot will start reducing the speed on one wheel.
      motors.setSpeeds(speed, speed - (encoders.getCountsLeft() - encoders.getCountsRight()));
    }

    if (encoders.getCountsLeft() + 10 < encoders.getCountsRight()) {  //If gyroscope angle is more than one degree off (in the other direction), the bot will start reducing the speed on the other wheel.
      motors.setSpeeds(speed - (encoders.getCountsLeft() - encoders.getCountsRight()), speed);
    }

    if (buttonC.getSingleDebouncedRelease()) {  //This button will break the loop.
      buttonC.waitForRelease();
      delay(100);
      break;
    }
  }
  motors.setSpeeds(0, 0);  //To stop whatever movement is going on after the loop is done.
}
void driveStraightGyro(int driveDistance) {  //Drive straight according to gyroscope. The driveDistance input is how long it will drive in counts.
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
  motors.setSpeeds(speed, speed);
  while (encoders.getCountsLeft() < driveDistance && encoders.getCountsRight() < driveDistance) {
    displayAngle();
    displayCounts();
    if (getTurnAngleInDegrees() >= 1) {  //If gyroscope angle is more than one degree off, the bot will start reducing the speed on one wheel.
      motors.setSpeeds(speed, speed - (getTurnAngleInDegrees() * 5));
    }

    if (getTurnAngleInDegrees() <= -1) {  //If gyroscope angle is more than one degree off (in the other direction), the bot will start reducing the speed on the other wheel.
      motors.setSpeeds(speed + (getTurnAngleInDegrees() * 5), speed);
    }

    if (buttonC.getSingleDebouncedRelease()) {  //This button will break the loop.
      buttonC.waitForRelease();
      delay(100);
      break;
    }
  }
  motors.setSpeeds(0, 0);  //To stop whatever movement is going on after the loop is done.
}
void driveStraightSpeed(int driveDistance) {  //Drive straight just by setting same speed on each wheel. The driveDistance input is how long it will drive in counts.
  motors.setSpeeds(speed, speed);
  while (encoders.getCountsLeft() < driveDistance && encoders.getCountsRight() < driveDistance) {
    displayAngle();
    displayCounts();

    if (buttonC.getSingleDebouncedRelease()) {  //This button will break the loop.
      buttonC.waitForRelease();
      delay(100);
      break;
    }
  }
  motors.setSpeeds(0, 0);  //To stop whatever movement is going on after the loop is done.
}
int getDistanceInCounts(int cm) {  //This function converts distance in cm to distance in counts. It requres countsToMeasureDistanceOf and measuredDistance.
  float countsPrDistance = countsToMeasureDistanceOf / measuredDistance;
  float DistanceInCounts = cm * countsPrDistance;
  return ((int)DistanceInCounts);
}
void printAngleChange() {
  lastAngle = getTurnAngleInDegrees();
  Serial.print((millis() - startTime) / 100);  //Prints time since gyro calibration.
  Serial.print("	");
  Serial.println(getTurnAngleInDegrees());  //Prints current angle.
}

void loop() {
  displayAngle();
  displayCounts();

  if (getTurnAngleInDegrees() != lastAngle) {
    printAngleChange();
  }

  if (buttonA.getSingleDebouncedRelease()) {
    buttonA.waitForRelease();
    turnSensorSetup();
    encoders.getCountsAndResetRight();
    encoders.getCountsAndResetLeft();
    startTime = millis();
  }

  if (buttonB.getSingleDebouncedRelease()) {  //Drives the amount of counts to be measured. When the distance in measured, the measuredDistance can be updated with that measurement.
    buttonB.waitForRelease();
    delay(300);
    driveStraightGyro(countsToMeasureDistanceOf);
  }

  if (buttonC.getSingleDebouncedRelease()) {  //Drives 40 cm, after the encoder calibration.
    buttonC.waitForRelease();
    delay(300);
    driveStraightGyro(getDistanceInCounts(40));
  }
}

//IMU STUFFF! ------------------------------- Code from Mathias Rehm with minor tweaks
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("delay"));

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(1000);
  display.gotoXY(0, 0);
  display.print(F("Gyro cal"));
  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < rounds; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / rounds;

  display.clear();
  turnSensorReset();
}
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}
int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // do some math and pointer magic to turn angle in seconds to angle in degree
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}
void turnSensorUpdate() {

  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}
